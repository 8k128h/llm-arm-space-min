#!/usr/bin/env python3
"""指令を1つ送り、完了を待ってから、掃引体積を保存する

使い方:
  python3 measure.py 名前 px py pz qx qy qz travel   … 記録して保存
  python3 measure.py - px py pz qx qy qz travel      … 記録せずに動かすだけ(準備用)
"""
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String


def spin_for(node, sec):
    end = time.time() + sec
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)


def send_ctrl(node, pub, text, times=3):
    for _ in range(times):  # 取りこぼし対策で数回送る(何回届いても結果は同じ)
        pub.publish(String(data=text))
        spin_for(node, 0.3)


def main():
    name = sys.argv[1]
    values = [float(v) for v in sys.argv[2:]]
    rclpy.init()
    node = Node('measure')
    done = []
    node.create_subscription(String, '/pick_place_done', lambda m: done.append(m.data), 10)
    ctrl = node.create_publisher(String, '/sweep_ctrl', 10)
    cmd = node.create_publisher(Float64MultiArray, '/pick_place_cmd', 10)

    # 相手のノードとつながるまで待つ
    t0 = time.time()
    while cmd.get_subscription_count() == 0 or (name != '-' and ctrl.get_subscription_count() == 0):
        if time.time() - t0 > 10:
            print('実行ノードか記録ノードが見つかりません')
            return
        spin_for(node, 0.1)
    spin_for(node, 0.5)

    if name != '-':
        send_ctrl(node, ctrl, 'reset')
        spin_for(node, 0.5)

    start = time.time()
    cmd.publish(Float64MultiArray(data=values))
    while not done and time.time() - start < 180:
        spin_for(node, 0.1)
    elapsed = time.time() - start
    print(f'結果: {done[0] if done else "タイムアウト"}  所要時間: {elapsed:.1f} 秒')

    if name != '-':
        spin_for(node, 1.0)  # 最後の動きを記録し終えるまで少し待つ
        send_ctrl(node, ctrl, f'save {name}')
        spin_for(node, 0.5)
        print('保存しました(体積は記録ノードの画面に表示されます)')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
