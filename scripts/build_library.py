#!/usr/bin/env python3
"""動作の部品ごとの掃引空間を、まとめて測ってライブラリにする(箱のないワールドで実行)"""
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String

L, C, R = (0.0, 0.1), (0.1, 0.0), (0.0, -0.1)
Z0, Z1 = 0.025, 0.055          # 1段目・2段目のつかむ点
LOW, HIGH = 0.033, 0.10        # 低いモード・傾けるモードの高さ


def putback(slot, z, travel):
    return [slot[0], slot[1], z, slot[0], slot[1], z, travel]


SEQUENCE = (
    [putback(s, Z0, LOW) for s in (L, C, R, L)] +    # 低いモード:上げ下げ、L-C, C-R, R-L の回転
    [putback(s, Z0, HIGH) for s in (L, C, R, L)] +   # 傾けるモード:同上
    [putback(s, Z1, HIGH) for s in (C, R, L)] +      # 傾けるモード:2段目の上げ下げ
    [putback(C, Z0, LOW), putback(C, Z0, HIGH)]      # 姿勢の切り替え
)


def spin_for(node, sec):
    end = time.time() + sec
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)


def send_ctrl(node, pub, text, times=3):
    for _ in range(times):  # 取りこぼし対策で数回送る(何回届いても結果は同じ)
        pub.publish(String(data=text))
        spin_for(node, 0.3)


def main():
    rclpy.init()
    node = Node('build_library')
    done = []
    node.create_subscription(String, '/pick_place_done', lambda m: done.append(m.data), 10)
    ctrl = node.create_publisher(String, '/sweep_ctrl', 10)
    cmd = node.create_publisher(Float64MultiArray, '/pick_place_cmd', 10)
    while cmd.get_subscription_count() == 0 or ctrl.get_subscription_count() == 0:
        spin_for(node, 0.2)
    spin_for(node, 0.5)

    send_ctrl(node, ctrl, 'reset_lib')
    spin_for(node, 0.5)
    for i, values in enumerate(SEQUENCE, 1):
        done.clear()
        cmd.publish(Float64MultiArray(data=[float(v) for v in values]))
        start = time.time()
        while not done and time.time() - start < 180:
            spin_for(node, 0.1)
        print(f'[{i}/{len(SEQUENCE)}] {done[0] if done else "タイムアウト"}  '
              f'({time.time() - start:.1f} 秒)')
    spin_for(node, 1.0)
    send_ctrl(node, ctrl, 'lib', 1)
    send_ctrl(node, ctrl, 'save_lib library')
    spin_for(node, 1.0)
    print('完了。部品ごとの体積は記録ノードの画面に表示されます')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
