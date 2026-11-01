// pick_place_server: /pick_place_cmd を受け取り、箱を掴んで置く実行ノード
//
// 指令 (std_msgs/Float64MultiArray, ロボット根元基準 [m]):
//   7要素: [掴む箱x, y, つかむ点z,  置く先x, y, つかむ点z,  運ぶ高さ]
//   8要素: 上に加えて [近づく高さ](省略時は運ぶ高さと同じ)
//          高さが TOPDOWN_MAX 以下 → 真下向きのまま / それより上 → 手先を傾けて高く
//   1要素: home 姿勢に戻る
//   3要素: [x, y, z] 真下向きのまま、まっすぐ動かす(実験用)
//   5要素: [x, y, z, 傾き(度), 0] その姿勢へ自由経路で動かす(実験用)
// 結果: /pick_place_done (std_msgs/String) に "ok ..." か "fail ..."
// 今動かしている部品の名前: /pick_place_step (std_msgs/String)
//
// 参考: crane_plus_examples/src/pick_and_place.cpp (RT Corporation, Apache-2.0)

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit_msgs/msg/robot_trajectory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using namespace std::chrono_literals;
static const rclcpp::Logger LOGGER = rclcpp::get_logger("pick_place_server");

double to_radians(double deg) {return deg * M_PI / 180.0;}

// ---- サンプル値と実験から決めた定数 ----
constexpr double HAND_OFFSET_Z = 0.115;  // つかむ点 → 手先リンクの距離(手先の軸方向)
constexpr double RADIAL_OFFSET = -0.01;  // 手先リンクと指の中心の水平方向のずれ
constexpr double PLACE_MARGIN = 0.005;   // 置くときは少し上で離す
constexpr double SAME_RADIUS_TOL = 0.002;
constexpr double TOPDOWN_MAX = 0.06;     // 真下向きで上げられる、つかむ点の高さの上限
const double HIGH_PITCH = to_radians(135);  // 高く運ぶときの手先の傾き
const double JOINT1_LIMIT = to_radians(150);
// 掴むときに根元を回す角度。固定側の指と箱の間に約5mmの隙間を作る(左回りが正)
const double PICK_YAW_OFFSET = to_radians(3);
// グリッパを開く角度[度](負が開く方向)。箱の幅より少し開く程度にして、隣の箱に当たらないようにする
constexpr double GRIPPER_OPEN_DEG = 4.0;
// 部品の名前を付けるためだけに使う値
constexpr double LABEL_GRASP = 0.025;
constexpr double LABEL_BOX = 0.03;

// 軌道の関節角度の変化量の合計 [rad]
double trajectory_cost(const trajectory_msgs::msg::JointTrajectory & traj)
{
  double cost = 0.0;
  const auto & pts = traj.points;
  for (size_t i = 1; i < pts.size(); ++i) {
    for (size_t j = 0; j < pts[i].positions.size(); ++j) {
      cost += std::abs(pts[i].positions[j] - pts[i - 1].positions[j]);
    }
  }
  return cost;
}

std::string slot_name(double x, double y)
{
  // 45度ごとの5か所: left(90), front_left(45), center(0), front_right(-45), right(-90)
  const long k = std::lround(std::atan2(y, x) / (M_PI / 4.0));
  if (k >= 2) {return "left";}
  if (k == 1) {return "front_left";}
  if (k == 0) {return "center";}
  if (k == -1) {return "front_right";}
  return "right";
}

std::string pair_name(std::string a, std::string b)
{
  if (b < a) {std::swap(a, b);}
  return a + "-" + b;
}

std::string level_of(double z)
{
  return std::to_string(std::max(0L, std::lround((z - LABEL_GRASP) / LABEL_BOX)));
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto arm_node = rclcpp::Node::make_shared("move_group_arm_node", node_options);
  auto gripper_node = rclcpp::Node::make_shared("move_group_gripper_node", node_options);
  auto io_node = rclcpp::Node::make_shared("pick_place_server");

  std::mutex mtx;
  std::condition_variable cv;
  std::deque<std::vector<double>> queue;
  auto sub = io_node->create_subscription<std_msgs::msg::Float64MultiArray>(
    "/pick_place_cmd", 10,
    [&](std_msgs::msg::Float64MultiArray::SharedPtr msg) {
      {
        std::lock_guard<std::mutex> lk(mtx);
        queue.push_back(msg->data);
      }
      cv.notify_one();
    });
  auto pub = io_node->create_publisher<std_msgs::msg::String>("/pick_place_done", 10);
  auto step_pub = io_node->create_publisher<std_msgs::msg::String>("/pick_place_step", 10);
  auto set_step = [&](const std::string & label) {
      std_msgs::msg::String m;
      m.data = label;
      step_pub->publish(m);
    };

  std::atomic<bool> naive_mode{false};
  auto mode_sub = io_node->create_subscription<std_msgs::msg::String>(
    "/pick_place_mode", 10,
    [&](std_msgs::msg::String::SharedPtr m) {
      naive_mode = (m->data == "naive");
      RCLCPP_INFO(LOGGER, "動作モード: %s", m->data.c_str());
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(arm_node);
  executor.add_node(gripper_node);
  executor.add_node(io_node);
  std::thread([&executor]() {executor.spin();}).detach();

  MoveGroupInterface arm(arm_node, "arm");
  arm.setMaxVelocityScalingFactor(1.0);
  arm.setMaxAccelerationScalingFactor(1.0);
  MoveGroupInterface gripper(gripper_node, "gripper");
  gripper.setMaxVelocityScalingFactor(1.0);
  gripper.setMaxAccelerationScalingFactor(1.0);
  const double GRIPPER_OPEN = to_radians(GRIPPER_OPEN_DEG);
  const double GRIPPER_CLOSE = to_radians(30);

  double total_cost = 0.0;
  std::optional<std::pair<double, double>> hand_xy;

  auto make_pose = [&](double x, double y, double z, double pitch) {
      const double yaw = std::atan2(y, x);
      const double r = std::hypot(x, y) + RADIAL_OFFSET;
      const double hr = r - HAND_OFFSET_Z * std::sin(pitch);
      geometry_msgs::msg::Pose pose;
      pose.position.x = hr * std::cos(yaw);
      pose.position.y = hr * std::sin(yaw);
      pose.position.z = z - HAND_OFFSET_Z * std::cos(pitch);
      tf2::Quaternion q;
      q.setRPY(0.0, pitch, yaw);
      pose.orientation = tf2::toMsg(q);
      return pose;
    };

  auto move_free = [&](double x, double y, double z, double pitch) -> bool {
      arm.setPoseTarget(make_pose(x, y, z, pitch));
      MoveGroupInterface::Plan plan;
      if (!arm.plan(plan)) {
        RCLCPP_ERROR(LOGGER, "計画失敗: (%.3f, %.3f, %.3f, %.0f度)",
          x, y, z, pitch * 180.0 / M_PI);
        return false;
      }
      total_cost += trajectory_cost(plan.trajectory_.joint_trajectory);
      if (!arm.execute(plan)) {return false;}
      hand_xy = std::make_pair(x, y);
      return true;
    };

  auto move_line = [&](double x, double y, double z, double pitch) -> bool {
      std::vector<geometry_msgs::msg::Pose> waypoints{make_pose(x, y, z, pitch)};
      moveit_msgs::msg::RobotTrajectory traj;
      const double fraction = arm.computeCartesianPath(waypoints, 0.005, 0.0, traj);
      if (fraction < 0.99) {
        RCLCPP_WARN(LOGGER, "直線経路を作れません(%.0f%%)。自由経路で代用", fraction * 100.0);
        return move_free(x, y, z, pitch);
      }
      total_cost += trajectory_cost(traj.joint_trajectory);
      if (!arm.execute(traj)) {return false;}
      hand_xy = std::make_pair(x, y);
      return true;
    };

  auto move_across = [&](double x0, double y0, double x1, double y1,
      double z, double pitch) -> bool {
      if (std::abs(std::hypot(x0, y0) - std::hypot(x1, y1)) > SAME_RADIUS_TOL) {
        return move_free(x1, y1, z, pitch);
      }
      const double delta = std::remainder(std::atan2(y1, x1) - std::atan2(y0, x0), 2.0 * M_PI);
      if (std::abs(delta) < 1e-6) {return true;}
      auto joints = arm.getCurrentJointValues();
      double target = joints[0] + delta;
      if (target > JOINT1_LIMIT) {target -= 2.0 * M_PI;}
      if (target < -JOINT1_LIMIT) {target += 2.0 * M_PI;}
      joints[0] = target;
      arm.setJointValueTarget(joints);
      MoveGroupInterface::Plan plan;
      if (!arm.plan(plan)) {
        RCLCPP_ERROR(LOGGER, "回転の計画失敗");
        return false;
      }
      total_cost += trajectory_cost(plan.trajectory_.joint_trajectory);
      if (!arm.execute(plan)) {return false;}
      hand_xy = std::make_pair(x1, y1);
      return true;
    };

  // 近づく高さ・姿勢で (x, y) の上へ行く
  auto go_above = [&](double x, double y, double travel, double pitch,
      const std::string & mode) -> bool {
      if (!hand_xy) {
        set_step("free");
        return move_free(x, y, travel, pitch);
      }
      const auto [hx, hy] = *hand_xy;
      const std::string cur = slot_name(hx, hy);
      set_step("adjust:" + cur);
      if (!move_line(hx, hy, travel, pitch)) {return false;}
      set_step("arc:" + pair_name(cur, slot_name(x, y)) + ":" + mode);
      return move_across(hx, hy, x, y, travel, pitch);
    };

  auto set_gripper = [&](double value) {
      auto joints = gripper.getCurrentJointValues();
      joints[0] = value;
      gripper.setJointValueTarget(joints);
      gripper.move();
    };

  // 比較用(naive モード): home 姿勢に戻る
  auto go_home = [&]() -> bool {
      set_step("naive");
      arm.setNamedTarget("home");
      MoveGroupInterface::Plan plan;
      if (!arm.plan(plan)) {return false;}
      total_cost += trajectory_cost(plan.trajectory_.joint_trajectory);
      const bool ok = static_cast<bool>(arm.execute(plan));
      hand_xy.reset();
      return ok;
    };

  set_step("free");
  arm.setNamedTarget("home");
  arm.move();
  set_gripper(GRIPPER_OPEN);
  set_step("idle");
  RCLCPP_INFO(LOGGER, "準備完了。/pick_place_cmd を待っています");

  while (rclcpp::ok()) {
    std::vector<double> d;
    {
      std::unique_lock<std::mutex> lk(mtx);
      cv.wait_for(lk, 100ms, [&] {return !queue.empty();});
      if (queue.empty()) {continue;}
      d = queue.front();
      queue.pop_front();
    }

    std_msgs::msg::String result;
    const double cost_before = total_cost;

    if (d.size() == 1) {
      set_step("free");
      arm.setNamedTarget("home");
      MoveGroupInterface::Plan plan;
      bool ok = static_cast<bool>(arm.plan(plan));
      if (ok) {
        total_cost += trajectory_cost(plan.trajectory_.joint_trajectory);
        ok = static_cast<bool>(arm.execute(plan));
      }
      hand_xy.reset();
      result.data = ok ? "ok home" : "fail home";
    } else if (d.size() == 3) {
      set_step("free");
      bool ok = move_line(d[0], d[1], d[2], M_PI);
      result.data = ok ? "ok line" : "fail line";
    } else if (d.size() == 5) {
      set_step("free");
      bool ok = move_free(d[0], d[1], d[2], to_radians(d[3]));
      result.data = ok ? "ok pose" : "fail pose";
    } else if (naive_mode && d.size() >= 7 && d.size() <= 9) {
      // 比較用: サンプルと同じく、1手ごとに home に戻る。運ぶ高さは安全のため常に傾けて高く
      const double yaw_off = d.size() == 9 ? to_radians(d[8]) : PICK_YAW_OFFSET;
      const double c = std::cos(yaw_off), s = std::sin(yaw_off);
      const double px = d[0] * c - d[1] * s, py = d[0] * s + d[1] * c, pz = d[2];
      const double qx = d[3], qy = d[4], qz = d[5];
      const double H = 0.10;
      std::string step;
      bool ok = true;
      set_step("naive");
      set_gripper(GRIPPER_OPEN);
      if (ok) {step = "home"; ok = go_home();}
      if (ok) {step = "pre_pick"; ok = move_free(px, py, H, HIGH_PITCH);}
      if (ok) {step = "pick"; ok = move_line(px, py, pz, M_PI);}
      if (ok) {
        set_gripper(GRIPPER_CLOSE);
        rclcpp::sleep_for(300ms);
        const double grip_deg = gripper.getCurrentJointValues()[0] * 180.0 / M_PI;
        RCLCPP_INFO(LOGGER, "掴んだときのグリッパ角度: %.1f 度", grip_deg);
        if (grip_deg > 27.0) {
          RCLCPP_ERROR(LOGGER, "掴みそこねました");
          step = "grasp";
          ok = false;
          set_gripper(GRIPPER_OPEN);
        }
      }
      if (ok) {step = "lift"; ok = move_line(px, py, H, HIGH_PITCH);}
      if (ok) {step = "home"; ok = go_home();}
      if (ok) {step = "pre_place"; ok = move_free(qx, qy, H, HIGH_PITCH);}
      if (ok) {step = "place"; ok = move_line(qx, qy, qz + PLACE_MARGIN, M_PI);}
      if (ok) {set_gripper(GRIPPER_OPEN);}
      if (ok) {step = "retreat"; ok = move_line(qx, qy, H, HIGH_PITCH);}
      if (ok) {step = "home"; ok = go_home();}
      char buf[160];
      if (ok) {
        std::snprintf(buf, sizeof(buf), "ok mode=naive cost=%.3f total=%.3f",
          total_cost - cost_before, total_cost);
      } else {
        std::snprintf(buf, sizeof(buf), "fail step=%s", step.c_str());
      }
      result.data = buf;
    } else if (d.size() < 7 || d.size() > 9) {
      result.data = "fail bad_command";
    } else {
      const std::string src = slot_name(d[0], d[1]);
      const std::string dst = slot_name(d[3], d[4]);
      // 掴む位置は、根元を PICK_YAW_OFFSET だけ回した向きにする
      // 9番目があれば、箱の実際の位置から計算した角度を使う
      const double yaw_off = d.size() == 9 ? to_radians(d[8]) : PICK_YAW_OFFSET;
      const double c = std::cos(yaw_off), s = std::sin(yaw_off);
      const double px = d[0] * c - d[1] * s, py = d[0] * s + d[1] * c, pz = d[2];
      const double qx = d[3], qy = d[4], qz = d[5];
      const double travel = d[6];                              // 運ぶ高さ
      const double approach = d.size() >= 8 ? d[7] : travel;   // 近づく高さ
      const bool high_c = travel > TOPDOWN_MAX + 1e-9;
      const bool high_a = approach > TOPDOWN_MAX + 1e-9;
      const double tc = high_c ? HIGH_PITCH : M_PI;
      const double ta = high_a ? HIGH_PITCH : M_PI;
      const std::string mc = high_c ? "high" : "low";
      const std::string ma = high_a ? "high" : "low";
      std::string step;
      bool ok = true;

      set_gripper(GRIPPER_OPEN);
      if (ok) {step = "pre_pick"; ok = go_above(px, py, approach, ta, ma);}
      if (ok) {
        step = "pick";
        set_step("vert:" + src + ":" + ma + ":" + level_of(pz));
        ok = move_line(px, py, pz, M_PI);
      }
      if (ok) {
        set_gripper(GRIPPER_CLOSE);
        rclcpp::sleep_for(300ms);
        const double grip_deg = gripper.getCurrentJointValues()[0] * 180.0 / M_PI;
        RCLCPP_INFO(LOGGER, "掴んだときのグリッパ角度: %.1f 度", grip_deg);
        if (grip_deg > 27.0) {  // ほぼ閉じきった = 指の間に箱がない
          RCLCPP_ERROR(LOGGER, "掴みそこねました");
          step = "grasp";
          ok = false;
          set_gripper(GRIPPER_OPEN);
        }
      }
      if (ok) {
        step = "lift";
        set_step("vert:" + src + ":" + mc + ":" + level_of(pz));
        ok = move_line(px, py, travel, tc);
      }
      if (ok) {
        step = "transfer";
        set_step("arc:" + pair_name(src, dst) + ":" + mc);
        ok = move_across(px, py, qx, qy, travel, tc);
      }
      if (ok) {
        step = "place";
        set_step("vert:" + dst + ":" + mc + ":" + level_of(qz));
        ok = move_line(qx, qy, qz + PLACE_MARGIN, M_PI);
      }
      if (ok) {set_gripper(GRIPPER_OPEN);}
      if (ok) {step = "retreat"; ok = move_line(qx, qy, travel, tc);}

      char buf[160];
      if (ok) {
        std::snprintf(buf, sizeof(buf), "ok mode=%s/%s cost=%.3f total=%.3f",
          ma.c_str(), mc.c_str(), total_cost - cost_before, total_cost);
      } else {
        std::snprintf(buf, sizeof(buf), "fail step=%s", step.c_str());
      }
      result.data = buf;
    }
    set_step("idle");
    RCLCPP_INFO(LOGGER, "%s", result.data.c_str());
    pub->publish(result);
  }

  rclcpp::shutdown();
  return 0;
}
