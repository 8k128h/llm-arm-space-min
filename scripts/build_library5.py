#!/usr/bin/env python3
"""5か所の置き場所で、動作の部品ごとの掃引空間を測ってライブラリにする(箱のないワールドで実行)"""
import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String

ANG = {"L": 90, "FL": 45, "C": 0, "FR": -45, "R": -90}
POS = {k: (round(0.1 * math.cos(math.radians(a)), 4), round(0.1 * math.sin(math.radians(a)), 4))
       for k, a in ANG.items()}
Z0, Z1 = 0.025, 0.055
LOW, HIGH = 0.033, 0.10
# 10通りの回転を、一筆書きで1回ずつ通る順番
TOUR = ["L", "FL", "C", "FR", "R", "L", "C", "R", "FL", "FR", "L"]


def putback(slot, z, travel):
    x, y = POS[slot]
    return [x, y, z, x, y, z, travel]


SEQUENCE = (
    [putback(s, Z0, LOW) for s in TOUR] +                                   # 低いモード
    [putback(s, Z0, HIGH) for s in TOUR] +                                  # 傾けるモード
    [c for s in ANG for c in (putback(s, Z0, LOW), putback(s, Z0, HIGH))] +  # 各場所での切り替え
    [putback(s, Z1, HIGH) for s in ANG]                                     # 2段目の上げ下げ
)


def spin_for(node, sec):
    end = time.time() + sec
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)


def main():
    rclpy.init()
    node = Node('build_library5')
    done = []
    node.create_subscription(String, '/pick_place_done', lambda m: done.append(m.data), 10)
    ctrl = node.create_publisher(String, '/sweep_ctrl', 10)
    cmd = node.create_publisher(Float64MultiArray, '/pick_place_cmd', 10)
    while cmd.get_subscription_count() == 0 or ctrl.get_subscription_count() == 0:
        spin_for(node, 0.2)
    spin_for(node, 0.5)

    def send_ctrl(text, times=3):
        for _ in range(times):
            ctrl.publish(String(data=text))
            spin_for(node, 0.3)

    send_ctrl('reset_lib')
    for i, values in enumerate(SEQUENCE, 1):
        done.clear()
        cmd.publish(Float64MultiArray(data=[float(v) for v in values]))
        start = time.time()
        while not done and time.time() - start < 180:
            spin_for(node, 0.1)
        print(f'[{i}/{len(SEQUENCE)}] {done[0] if done else "タイムアウト"}  '
              f'({time.time() - start:.1f} 秒)', flush=True)
    spin_for(node, 1.0)
    send_ctrl('lib', 1)
    send_ctrl('save_lib library5')
    print('完了。~/sweeps/library5.json に保存されます')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
