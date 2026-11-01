#!/usr/bin/env python3
"""計画器で手順を作り、実行ノードに順に送って実行する

使い方:
  python3 run_plan.py '<指示のJSON>' [--objective balanced|volume|moves] [--batch] [--naive]
                       [--record 名前] [--tag ラベル] [--reset-state] [--dry-run] [--no-feedback]
指示のJSON: 指示(条件のリスト)のリスト
  例: '[[{"type": "at", "box": "blue", "slot": "front_right"}]]'
--batch を付けると指示をまとめて計画(セット)、付けなければ1つずつ計画・実行(逐次)。
--reset-state: Gazebo の箱を初期位置に戻し、ロボットを待機姿勢(中央の上)にしてから始める。
--naive: サンプルと同じ動かし方(1手ごとに home に戻る)で、手数最小の手順を実行する(比較用)。
今の配置などは ~/sweeps/state.json に、結果は ~/sweeps/results.jsonl に記録する。
"""
import argparse
import json
import math
import os
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import planner as P  # noqa: E402
import poses as PS  # noqa: E402

SWEEPS = os.path.expanduser("~/sweeps")
STATE_PATH = os.path.join(SWEEPS, "state.json")
RESULTS_PATH = os.path.join(SWEEPS, "results.jsonl")
INITIAL = {"left": ["red"], "center": ["green"], "right": ["blue"]}
INITIAL_POSES = {"red": (-0.022, 0.1), "green": (0.1, 0.022), "blue": (0.022, -0.1)}  # 着地位置
BOX_Z = 1.031
STANDBY = [0.1, 0.0, 0.10, 135.0, 0.0]   # 待機姿勢: 中央の上、傾けた姿勢
GAP = 0.0052        # 掴むとき、固定側の指と箱の間に空ける隙間 [m]
FINGER_R = 0.1      # 指の位置の、根元からの距離 [m]


def pick_yaw(box, slot):
    """箱の実際の位置から、掴むときに根元を回す角度[度]と、左回りのずれ[mm]を求める"""
    x, y, _, _ = PS.read_poses()[f"{box}_box"]
    _, dt = PS.offsets(x, y, P.ALL_SLOT_ANGLES[slot])
    return math.degrees((GAP + dt) / FINGER_R), 1000 * dt


def reset_boxes():
    for color, (x, y) in INITIAL_POSES.items():
        req = f'name: "{color}_box", position: {{x: {x}, y: {y}, z: {BOX_Z}}}, orientation: {{w: 1}}'
        subprocess.run(["ign", "service", "-s", "/world/default/set_pose",
                        "--reqtype", "ignition.msgs.Pose", "--reptype", "ignition.msgs.Boolean",
                        "--timeout", "3000", "--req", req],
                       capture_output=True, text=True, timeout=15)


def load_state(reset):
    stacks = P.empty_stacks()
    if reset or not os.path.exists(STATE_PATH):
        stacks.update(INITIAL)
        return {"stacks": stacks, "hand": None, "hand_mode": "high", "used": []}
    with open(STATE_PATH) as f:
        st = json.load(f)
    for s, v in st["stacks"].items():
        if s in stacks:
            stacks[s] = v
    st["stacks"] = stacks
    return st


def save_state(st):
    os.makedirs(SWEEPS, exist_ok=True)
    with open(STATE_PATH, "w") as f:
        json.dump(st, f, ensure_ascii=False)


class Robot:
    """実行ノード・記録ノードとのやりとり"""
    def __init__(self):
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import Float64MultiArray, String
        self.rclpy, self.Float64MultiArray, self.String = rclpy, Float64MultiArray, String
        rclpy.init()
        self.node = Node("run_plan")
        self.done = []
        self.node.create_subscription(String, "/pick_place_done", lambda m: self.done.append(m.data), 10)
        self.cmd_pub = self.node.create_publisher(Float64MultiArray, "/pick_place_cmd", 10)
        self.ctrl_pub = self.node.create_publisher(String, "/sweep_ctrl", 10)
        self.mode_pub = self.node.create_publisher(String, "/pick_place_mode", 10)
        t0 = time.time()
        while self.cmd_pub.get_subscription_count() == 0:
            if time.time() - t0 > 10:
                raise RuntimeError("実行ノードが見つかりません")
            self.spin_for(0.2)
        self.spin_for(0.5)

    def spin_for(self, sec):
        end = time.time() + sec
        while time.time() < end:
            self.rclpy.spin_once(self.node, timeout_sec=0.05)

    def repeat(self, pub, text, times=3):
        for _ in range(times):
            pub.publish(self.String(data=text))
            self.spin_for(0.3)

    def ctrl(self, text):
        self.repeat(self.ctrl_pub, text)

    def mode(self, text):
        self.repeat(self.mode_pub, text)

    def send(self, values):
        self.done.clear()
        self.cmd_pub.publish(self.Float64MultiArray(data=[float(v) for v in values]))
        start = time.time()
        while not self.done and time.time() - start < 300:
            self.spin_for(0.1)
        return (self.done[0] if self.done else "タイムアウト"), time.time() - start

    def close(self):
        self.node.destroy_node()
        self.rclpy.shutdown()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("stages")
    ap.add_argument("--objective", default="balanced", choices=["volume", "balanced", "moves"])
    ap.add_argument("--slots", type=int, default=5, choices=[3, 5])
    ap.add_argument("--lib", default=None)
    ap.add_argument("--record", default=None)
    ap.add_argument("--tag", default="")
    ap.add_argument("--batch", action="store_true")
    ap.add_argument("--naive", action="store_true")
    ap.add_argument("--reset-state", action="store_true")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--no-feedback", action="store_true")
    args = ap.parse_args()
    objective = "moves" if args.naive else args.objective

    names = (["left", "front_left", "center", "front_right", "right"] if args.slots == 5
             else ["left", "center", "right"])
    P.configure(names)
    lib = args.lib or os.path.join(SWEEPS, "library5.json" if args.slots == 5 else "library.json")
    vol = P.Volume(P.load_library(lib))
    st = load_state(args.reset_state)
    stages = json.loads(args.stages)

    robot = None if args.dry_run else Robot()
    if args.reset_state:
        if robot:
            print("箱を初期位置に戻し、待機姿勢に移動します")
            robot.mode("smart")
            reset_boxes()
            res, _ = robot.send(STANDBY)
            robot.spin_for(1.0)
            reset_boxes()  # 待機姿勢への移動で触れた場合に備えて、もう一度戻す
            robot.spin_for(1.0)
            print("  待機姿勢:", res)
        st.update({"hand": "center", "hand_mode": "high", "used": []})
        save_state(st)
    print("今の配置:", P.describe(st["stacks"]))

    if robot and args.record:
        robot.ctrl("reset")
    if robot and args.naive:
        robot.mode("naive")

    used = frozenset(st["used"])
    exec_time, n_moves, ok, predicted = 0.0, 0, True, None

    def execute(r):
        nonlocal exec_time, n_moves
        for step in r["steps"]:
            if "cmd" not in step:
                continue
            cmd = list(step["cmd"])
            if not args.no_feedback:
                yaw, dt = pick_yaw(step["box"], step["src"])
                cmd.append(yaw)
                print(f"  位置補正: 左回りに {dt:+.1f} mm → 根元を {yaw:.1f} 度回して掴む")
            res, sec = robot.send(cmd)
            exec_time += sec
            print(f"  実行: {P.JP[step['box']]} {P.JP[step['src']]}→{P.JP[step['dst']]}  "
                  f"{res}  ({sec:.1f} 秒)")
            if not res.startswith("ok"):
                return False
            n_moves += 1
            st["stacks"][step["dst"]].append(st["stacks"][step["src"]].pop())
            if args.naive:
                st.update({"hand": None, "hand_mode": "high"})
            else:
                st.update({"hand": step["dst"], "hand_mode": step["mc"]})
            save_state(st)
        return True

    try:
        if args.batch:
            r = P.plan(st["stacks"], stages, vol, objective, st["hand"], st["hand_mode"], used,
                       max_moves=2 * len(stages) + 2)
            plans = [r]
        else:
            plans = None
        for i in range(1 if args.batch else len(stages)):
            if not args.batch:
                r = P.plan(st["stacks"], [stages[i]], vol, objective, st["hand"], st["hand_mode"], used)
            if r is None:
                print("腕の制約内では実現できません。中止します")
                ok = False
                break
            P.show("計画(まとめて)" if args.batch else f"指示{i + 1}の計画", r)
            predicted = r["volume"]
            if args.dry_run:
                st.update({"stacks": r["final"], "hand": r["hand"], "hand_mode": r["hand_mode"]})
            elif not execute(r):
                print("失敗したので中止します")
                ok = False
                break
            used = r["parts"]
            st["used"] = sorted(used)
            save_state(st)
    finally:
        if robot:
            if args.naive:
                robot.mode("smart")
            if args.record:
                robot.spin_for(1.0)
                robot.ctrl(f"save {args.record}")
            robot.close()

    print("最終配置:", P.describe(st["stacks"]))
    if not args.dry_run:
        rec = {"tag": args.tag, "record": args.record, "objective": objective, "batch": args.batch,
               "naive": args.naive, "stages": stages, "ok": ok, "moves": n_moves,
               "exec_time": round(exec_time, 1),
               "predicted": None if args.naive else predicted}
        os.makedirs(SWEEPS, exist_ok=True)
        with open(RESULTS_PATH, "a") as f:
            f.write(json.dumps(rec, ensure_ascii=False) + "\n")
        print(f"実行時間 {exec_time:.1f} 秒 / {n_moves} 手"
              + ("" if args.naive or predicted is None else f" / 予測した空間 {predicted} cm3"))


if __name__ == "__main__":
    main()
