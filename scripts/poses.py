#!/usr/bin/env python3
"""Gazebo から箱の位置を読み、置き場所に対するずれを求める

単独で実行すると、各箱の置き場所・段・ずれ(外向き/左回り)・向きのずれを表示する。
run_plan.py からは read_poses() と offsets() を使う。
"""
import math
import re
import subprocess

ROBOT_Z = 1.015      # ロボットの根元の高さ(世界座標)
BOX = 0.03
R, T = 0.1, 0.022    # 置き場所の半径、グリッパによる横方向のずれ(着地位置)
ANGLES = {"left": 90, "front_left": 45, "center": 0, "front_right": -45, "right": -90}
JP = {"left": "左", "front_left": "斜め左", "center": "中央", "front_right": "斜め右", "right": "右"}


def read_poses():
    """箱の名前 → (x, y, z, 向き[度])"""
    out = subprocess.run(["ign", "topic", "-e", "-t", "/world/default/pose/info", "-n", "1"],
                         capture_output=True, text=True, timeout=30).stdout
    pat = re.compile(r'name: "(\w+_box)".*?position \{(.*?)\}.*?orientation \{(.*?)\}', re.S)
    poses = {}
    for name, pos, ori in pat.findall(out):
        v = {k: float(x) for k, x in re.findall(r'(\w+): ([-\d.e]+)', pos)}
        q = {k: float(x) for k, x in re.findall(r'(\w+): ([-\d.e]+)', ori)}
        yaw = math.degrees(2 * math.atan2(q.get("z", 0.0), q.get("w", 1.0)))
        poses[name] = (v.get("x", 0.0), v.get("y", 0.0), v.get("z", 0.0), yaw)
    return poses


def landing(deg):
    t = math.radians(deg)
    return (R * math.cos(t) - T * math.sin(t), R * math.sin(t) + T * math.cos(t))


def offsets(x, y, deg):
    """置き場所(角度 deg)の着地位置に対するずれ [m]: (外向き, 左回り)"""
    lx, ly = landing(deg)
    t = math.radians(deg)
    dr = (x - lx) * math.cos(t) + (y - ly) * math.sin(t)
    dt = -(x - lx) * math.sin(t) + (y - ly) * math.cos(t)
    return dr, dt


def nearest_slot(x, y):
    return min(ANGLES, key=lambda n: math.hypot(x - landing(ANGLES[n])[0], y - landing(ANGLES[n])[1]))


if __name__ == "__main__":
    print(f"{'箱':10s} {'置き場所':6s} {'段':>2s} {'ずれ[mm]':>8s} {'外向き[mm]':>9s} "
          f"{'左回り[mm]':>9s} {'向き[度]':>7s}")
    for name, (x, y, z, yaw) in sorted(read_poses().items()):
        slot = nearest_slot(x, y)
        dr, dt = offsets(x, y, ANGLES[slot])
        level = round((z - ROBOT_Z - BOX / 2) / BOX) + 1
        yaw_err = (yaw - ANGLES[slot] + 45) % 90 - 45
        print(f"{name:10s} {JP[slot]:6s} {level:2d} {1000 * math.hypot(dr, dt):8.1f} "
              f"{1000 * dr:9.1f} {1000 * dt:9.1f} {yaw_err:7.1f}")
