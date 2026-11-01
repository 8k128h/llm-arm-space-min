#!/usr/bin/env python3
"""ランダムな指示の組で、目的(空間最小/バランス/手数最小)と最適化の仕方(セット/逐次)を統計的に比べる

使い方: python3 eval_random.py [組の数] [乱数の種]
"""
import os
import random
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import planner as P  # noqa: E402

P.configure(["left", "front_left", "center", "front_right", "right"])
vol = P.Volume(P.load_library(os.path.expanduser(sys.argv[3] if len(sys.argv) > 3 else "~/sweeps/library5.json")))
SIDES = {"右側": ["front_right", "right"], "左側": ["left", "front_left"]}
HAND = "center"  # 作業開始時、手は中央の上で待機している(傾けた姿勢)
init = P.empty_stacks()
init.update({"left": ["red"], "center": ["green"], "right": ["blue"]})


def random_instruction(rng):
    """人が言いそうな1つの指示(条件1つ)をランダムに作る"""
    box = rng.choice(P.BOXES)
    kind = rng.choice(["at", "in", "on", "on_table"])
    if kind == "at":
        slot = rng.choice(P.SLOT_ORDER)
        return f"{P.JP[box]}を{P.JP[slot]}に", {"type": "at", "box": box, "slot": slot}
    if kind == "in":
        side = rng.choice(list(SIDES))
        return f"{P.JP[box]}を{side}に", {"type": "in", "box": box, "slots": SIDES[side]}
    if kind == "on":
        under = rng.choice([b for b in P.BOXES if b != box])
        return f"{P.JP[box]}を{P.JP[under]}の上に", {"type": "on", "box": box, "under": under}
    return f"{P.JP[box]}をテーブルに", {"type": "on_table", "box": box}


def main():
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 30
    rng = random.Random(int(sys.argv[2]) if len(sys.argv) > 2 else 0)
    methods = {
        "手数最小・セット": lambda st: P.plan(init, st, vol, "moves", hand=HAND, max_moves=4),
        "空間最小・セット": lambda st: P.plan(init, st, vol, "volume", hand=HAND, max_moves=4),
        "空間最小・逐次": lambda st: P.plan_online(init, st, vol, "volume", hand=HAND),
        "バランス・セット": lambda st: P.plan(init, st, vol, "balanced", hand=HAND, max_moves=4),
    }
    records = []
    t0 = time.time()
    while len(records) < n:
        k = rng.choice([2, 3])
        insts = [random_instruction(rng) for _ in range(k)]
        stages = [[c] for _, c in insts]
        base = methods["手数最小・セット"](stages)
        if base is None or base["moves"] == 0:
            continue  # 実現できない・何もしなくてよい組は除く
        res = {"手数最小・セット": base}
        for name, f in methods.items():
            if name not in res:
                res[name] = f(stages)
        records.append((" → ".join(t for t, _ in insts), res))
        print(f"[{len(records)}/{n}] {records[-1][0]}  ({time.time() - t0:.0f} 秒経過)", flush=True)

    print("\n===== 結果(手数最小・セットを基準にした比) =====")
    print(f"{'方法':14s} {'空間の差の平均':>12s} {'時間の差の平均':>12s} {'空間が減った割合':>14s}")
    for name in methods:
        dv, dt, better = [], [], 0
        for _, res in records:
            b, r = res["手数最小・セット"], res[name]
            if r is None:
                continue
            dv.append(r["volume"] / b["volume"] - 1)
            dt.append(r["time"] / b["time"] - 1)
            better += r["volume"] < b["volume"]
        print(f"{name:14s} {100 * sum(dv) / len(dv):+11.1f}% {100 * sum(dt) / len(dt):+11.1f}% "
              f"{100 * better / len(records):13.0f}%")

    diff = [(t, res) for t, res in records
            if res["空間最小・セット"]["volume"] < res["空間最小・逐次"]["volume"]]
    print(f"\nセットが逐次より空間を減らせた組: {len(diff)} / {len(records)}")
    for t, res in diff[:5]:
        print(f"  {t}: セット {res['空間最小・セット']['volume']} / 逐次 {res['空間最小・逐次']['volume']} cm3")


if __name__ == "__main__":
    main()
