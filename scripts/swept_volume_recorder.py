#!/usr/bin/env python3
"""アームが通った空間(掃引領域)を記録・可視化するノード

TF から各リンクの位置を読み、アームを線分で近似する。
線分から RADIUS 以内にある、一辺 VOXEL の立方体を「通った空間」として塗る。
和集合なので、同じ場所を何度通っても体積は増えない。
/pick_place_step で知らされる部品の名前ごとにも、空間を貯める(ライブラリ)。
gazebo_markers:=true のときは、Gazebo の画面にも通った空間を描く(表示専用、物理演算には影響しない)。

/sweep_ctrl (std_msgs/String) で操作:
  "reset"            全体の記録を消す(ライブラリは残る)
  "report"           全体の体積を表示
  "save 名前"        全体の記録を ~/sweeps/名前.npy に保存
  "lib"              部品ごとの体積を一覧表示
  "reset_lib"        ライブラリを消す
  "save_lib 名前"    ライブラリを ~/sweeps/名前.json に保存
"""
import json
import math
import os
import subprocess
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from tf2_ros import Buffer, TransformListener

VOXEL = 0.01        # 立方体の一辺 [m]
RADIUS = 0.015      # 線分の周りの太さ(腕の太さの近似)[m]
STEP = 0.005        # 線分・補間の刻み [m]
MAX_INTERP = 15     # 1回の補間の上限
FINGER_LEN = 0.135  # 手先リンクの原点から指先までの距離 [m]
OFFSET = 512
IGNORE = ('idle',)

_r = int(math.ceil(RADIUS / VOXEL))
SPHERE = np.array([(i, j, k)
                   for i in range(-_r, _r + 1)
                   for j in range(-_r, _r + 1)
                   for k in range(-_r, _r + 1)
                   if math.sqrt(i * i + j * j + k * k) * VOXEL <= RADIUS + 1e-9])


def encode(idx):
    i, j, k = (idx + OFFSET).T
    return (i.astype(np.int64) << 20) | (j.astype(np.int64) << 10) | k.astype(np.int64)


def decode(keys):
    keys = np.asarray(keys, dtype=np.int64)
    return np.stack([(keys >> 20) & 1023, (keys >> 10) & 1023, keys & 1023], axis=1) - OFFSET


def z_axis(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)])


def cm3(n):
    return n * (VOXEL * 100) ** 3


class Recorder(Node):
    def __init__(self):
        super().__init__('swept_volume_recorder')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('chain', ['crane_plus_base', 'crane_plus_link1', 'crane_plus_link2',
                                         'crane_plus_link3', 'crane_plus_link4'])
        self.declare_parameter('ee_link', 'crane_plus_link4')
        self.declare_parameter('finger_link', 'crane_plus_link_hand')
        self.declare_parameter('gazebo_markers', False)   # Gazebo にも通過した空間を描く
        self.declare_parameter('gz_cell', 0.02)           # Gazebo に描く立方体の一辺 [m]
        self.declare_parameter('gz_rgba', '0.1 0.1 0.1 0.3')   # Gazebo に描く色(赤 緑 青 不透明度)
        self.declare_parameter('robot_z', 1.015)          # ロボットの根元の高さ(世界座標)
        self.base = self.get_parameter('base_frame').value
        self.chain = list(self.get_parameter('chain').value)
        self.ee = self.get_parameter('ee_link').value
        self.finger = self.get_parameter('finger_link').value
        self.gz_on = bool(self.get_parameter('gazebo_markers').value)
        self.gz_cell = float(self.get_parameter('gz_cell').value)
        self.robot_z = float(self.get_parameter('robot_z').value)
        self.gz_rgba = [float(v) for v in str(self.get_parameter('gz_rgba').value).split()]

        self.buf = Buffer()
        self.tl = TransformListener(self.buf, self)
        self.lock = threading.Lock()
        self.voxels = set()
        self.lib = {}
        self.label = 'idle'
        self.prev = None
        self.warned = False
        self.marker_n = -1
        self.marker_points = []
        self.gz_sent, self.gz_next_id, self.gz_gen, self.gz_mode = set(), 1, 0, 'array'

        self.cb_sub = MutuallyExclusiveCallbackGroup()
        self.cb_sample = MutuallyExclusiveCallbackGroup()
        self.cb_pub = MutuallyExclusiveCallbackGroup()
        self.cb_gz = MutuallyExclusiveCallbackGroup()
        self.pub = self.create_publisher(Marker, '/swept_volume_markers', 10)
        self.create_subscription(String, '/sweep_ctrl', self.on_ctrl, 10,
                                 callback_group=self.cb_sub)
        self.create_subscription(String, '/pick_place_step', self.on_step, 10,
                                 callback_group=self.cb_sub)
        self.create_timer(1.0 / 30.0, self.sample, callback_group=self.cb_sample)
        self.create_timer(0.5, self.publish, callback_group=self.cb_pub)
        self.create_timer(1.0, self.gz_update, callback_group=self.cb_gz)
        self.get_logger().info('掃引領域の記録を開始しました(ライブラリ対応版'
                               + (' / Gazebo表示あり' if self.gz_on else '') + ')')

    def on_step(self, msg):
        self.label = msg.data

    # ---- アームの位置を読んで、通った立方体を塗る ----
    def lookup(self, frame):
        t = self.buf.lookup_transform(self.base, frame, rclpy.time.Time())
        p = t.transform.translation
        return np.array([p.x, p.y, p.z]), t.transform.rotation

    def keypoints(self):
        pts = [self.lookup(f)[0] for f in self.chain]
        ee_pos, ee_rot = self.lookup(self.ee)
        pts.append(ee_pos + FINGER_LEN * z_axis(ee_rot))   # 指先
        pts.append(self.lookup(self.finger)[0])            # 動く指
        return np.array(pts)

    def segments(self, pts):
        n = len(self.chain)
        segs = [(pts[i], pts[i + 1]) for i in range(n - 1)]
        ee_i = self.chain.index(self.ee) if self.ee in self.chain else n - 1
        segs.append((pts[ee_i], pts[n]))
        segs.append((pts[ee_i], pts[n + 1]))
        return segs

    def paint(self, pts):
        centers = []
        for a, b in self.segments(pts):
            k = max(1, int(math.ceil(np.linalg.norm(b - a) / STEP)))
            t = np.linspace(0.0, 1.0, k + 1)[:, None]
            centers.append(a + (b - a) * t)
        idx = np.floor(np.vstack(centers) / VOXEL).astype(np.int64)
        idx = (idx[:, None, :] + SPHERE[None, :, :]).reshape(-1, 3)
        keys = np.unique(encode(idx)).tolist()
        with self.lock:
            self.voxels.update(keys)
            if self.label not in IGNORE:
                self.lib.setdefault(self.label, set()).update(keys)

    def sample(self):
        try:
            pts = self.keypoints()
        except Exception as e:
            if not self.warned:
                self.get_logger().warn(f'TF を読めません(待機中): {e}')
                self.warned = True
            return
        if self.warned:
            self.get_logger().info('TF を受信しました。記録中です')
            self.warned = False
        if self.prev is None:
            self.paint(pts)
        else:
            move = np.max(np.linalg.norm(pts - self.prev, axis=1))
            if move < 1e-4:
                return
            n = min(MAX_INTERP, max(1, int(math.ceil(move / STEP))))
            for i in range(1, n + 1):
                self.paint(self.prev + (pts - self.prev) * (i / n))
        self.prev = pts

    # ---- RViz への表示 ----
    def publish(self):
        m = Marker()
        m.header.frame_id = self.base
        m.ns, m.id = 'swept', 0
        m.type, m.action = Marker.CUBE_LIST, Marker.ADD
        m.scale.x = m.scale.y = m.scale.z = VOXEL
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.5, 0.0, 0.25
        m.pose.orientation.w = 1.0
        with self.lock:
            n = len(self.voxels)
            keys = list(self.voxels) if n != self.marker_n else None
        if keys is not None:
            self.marker_n = n
            c = (decode(keys) + 0.5) * VOXEL if keys else []
            self.marker_points = [Point(x=float(x), y=float(y), z=float(z)) for x, y, z in c]
        m.points = self.marker_points
        self.pub.publish(m)

    # ---- Gazebo への表示(表示専用の図形。物理演算には影響しない) ----
    def gz_cmd(self, service, reqtype, reptype, req):
        return ["ign", "service", "-s", service, "--reqtype", reqtype,
                "--reptype", reptype, "--timeout", "3000", "--req", req]

    def gz_marker(self, i, x, y, z, single=False):
        c = self.gz_cell
        body = (f'action: ADD_MODIFY ns: "swept" id: {i} type: BOX '
                f'scale {{ x: {c} y: {c} z: {c} }} '
                f'pose {{ position {{ x: {x:.4f} y: {y:.4f} z: {z:.4f} }} }} '
                f'material {{ ambient {{ {self.gz_color()} }} '
                f'diffuse {{ {self.gz_color()} }} }}')
        return body if single else f"marker {{ {body} }}"

    def gz_color(self):
        r, g, b, a = self.gz_rgba
        return f'r: {r} g: {g} b: {b} a: {a}'

    def gz_send_array(self, items):
        """まとめて描く(/marker_array)。成功したら True"""
        req = " ".join(self.gz_marker(i, x, y, z) for i, x, y, z in items)
        r = subprocess.run(self.gz_cmd("/marker_array", "ignition.msgs.Marker_V",
                                       "ignition.msgs.Boolean", req),
                           capture_output=True, text=True, timeout=20)
        return "data: true" in r.stdout

    def gz_send_single(self, items):
        """1つずつ描く(/marker)。16個ずつ同時に送る"""
        for k in range(0, len(items), 16):
            procs = [subprocess.Popen(self.gz_cmd("/marker", "ignition.msgs.Marker",
                                                  "ignition.msgs.Empty",
                                                  self.gz_marker(i, x, y, z, single=True)),
                                      stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                     for i, x, y, z in items[k:k + 16]]
            for pr in procs:
                pr.wait(timeout=20)

    def gz_update(self):
        if not self.gz_on:
            return
        with self.lock:
            keys = list(self.voxels)
            gen = self.gz_gen
        if not keys:
            return
        c = (decode(keys) + 0.5) * VOXEL
        cells = set(map(tuple, np.floor(c / self.gz_cell).astype(np.int64).tolist()))
        new = sorted(cells - self.gz_sent)
        cell = self.gz_cell
        for k in range(0, len(new), 250):
            if gen != self.gz_gen:  # 途中でリセットされたら送らない
                return
            chunk = new[k:k + 250]
            items = [(self.gz_next_id + j, (a + 0.5) * cell, (b + 0.5) * cell,
                      (d + 0.5) * cell + self.robot_z) for j, (a, b, d) in enumerate(chunk)]
            if self.gz_mode == 'array' and not self.gz_send_array(items):
                self.gz_mode = 'single'
                self.get_logger().warn('まとめて描く命令が使えないので、1つずつ描く方法に切り替えます')
            if self.gz_mode == 'single':
                self.gz_send_single(items)
            self.gz_next_id += len(chunk)
            self.gz_sent.update(chunk)
        if new:
            self.get_logger().info(f'Gazebo に描いた立方体: {len(self.gz_sent)} 個(+{len(new)})')

    def gz_clear(self):
        if not self.gz_on:
            return
        with self.lock:
            self.gz_gen += 1
            self.gz_sent = set()
        subprocess.run(self.gz_cmd("/marker", "ignition.msgs.Marker", "ignition.msgs.Empty",
                                   'action: DELETE_ALL ns: "swept"'),
                       capture_output=True, text=True, timeout=20)

    # ---- 操作 ----
    def on_ctrl(self, msg):
        cmd = msg.data.split(maxsplit=1)
        if not cmd:
            return
        d = os.path.expanduser('~/sweeps')
        if cmd[0] == 'reset':
            with self.lock:
                self.voxels.clear()
                self.prev = None
            self.gz_clear()
            self.get_logger().info('記録をリセットしました')
        elif cmd[0] == 'report':
            self.get_logger().info(f'掃引体積: {cm3(len(self.voxels)):.0f} cm3')
        elif cmd[0] == 'save' and len(cmd) == 2:
            os.makedirs(d, exist_ok=True)
            path = os.path.join(d, cmd[1] + '.npy')
            with self.lock:
                data = np.array(sorted(self.voxels), dtype=np.int64)
            np.save(path, data)
            self.get_logger().info(f'保存しました: {path} ({cm3(len(data)):.0f} cm3)')
        elif cmd[0] == 'reset_lib':
            with self.lock:
                self.lib.clear()
            self.get_logger().info('ライブラリをリセットしました')
        elif cmd[0] == 'lib':
            with self.lock:
                lines = [f'  {k:28s} {cm3(len(v)):6.0f} cm3' for k, v in sorted(self.lib.items())]
            self.get_logger().info('部品ごとの体積:\n' + ('\n'.join(lines) if lines else '  (まだありません)'))
        elif cmd[0] == 'save_lib' and len(cmd) == 2:
            os.makedirs(d, exist_ok=True)
            path = os.path.join(d, cmd[1] + '.json')
            with self.lock:
                data = {k: sorted(v) for k, v in self.lib.items()}
            with open(path, 'w') as f:
                json.dump(data, f)
            self.get_logger().info(f'ライブラリを保存しました: {path} ({len(data)} 部品)')


def main():
    rclpy.init()
    node = Recorder()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
