#!/usr/bin/env python3
"""指示の組 × 方法の組み合わせを、Gazebo で順に実行して記録する
(Gazebo・実行ノード・記録ノードを起動した状態で実行する)"""
import json
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
R_SIDE, L_SIDE = ["front_right", "right"], ["left", "front_left"]

CASES = {
    "c1": [[{"type": "in", "box": "red", "slots": R_SIDE}],
           [{"type": "on", "box": "green", "under": "blue"}]],
    "c2": [[{"type": "in", "box": "blue", "slots": L_SIDE}],
           [{"type": "at", "box": "green", "slot": "front_right"}]],
    "c3": [[{"type": "at", "box": "green", "slot": "front_right"}],
           [{"type": "on", "box": "red", "under": "blue"}],
           [{"type": "in", "box": "blue", "slots": R_SIDE}]],
}
METHODS = {
    "naive": ["--naive"],
    "moves": ["--objective", "moves", "--batch"],
    "volume": ["--objective", "volume", "--batch"],
    "volume_online": ["--objective", "volume"],
    "balanced": ["--objective", "balanced", "--batch"],
}


def main():
    only = sys.argv[1:]  # 例: python3 experiment.py c1   (指定した組だけ実行)
    t0 = time.time()
    runs = [(c, m) for c in CASES for m in METHODS if not only or c in only]
    for i, (case, method) in enumerate(runs, 1):
        name = f"exp_{case}_{method}"
        print(f"\n===== [{i}/{len(runs)}] {name}  ({(time.time() - t0) / 60:.0f} 分経過) =====",
              flush=True)
        cmd = [sys.executable, os.path.join(HERE, "run_plan.py"), "--reset-state",
               "--record", name, "--tag", case] + METHODS[method] + [json.dumps(CASES[case])]
        subprocess.run(cmd)
    print(f"\n完了({(time.time() - t0) / 60:.0f} 分)。python3 analyze.py で集計できます")


if __name__ == "__main__":
    main()
