#!/usr/bin/env python3
"""5か所の置き場所で、目的(空間最小/手数最小)と最適化の仕方(セット/逐次)で差が出るかを比べる"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import planner as P  # noqa: E402

P.configure(["left", "front_left", "center", "front_right", "right"])
vol = P.Volume(P.load_library(os.path.expanduser("~/sweeps/library5_synth.json")))

RIGHT_SIDE = ["front_right", "right"]
LEFT_SIDE = ["left", "front_left"]
init = P.empty_stacks()
init.update({"left": ["red"], "center": ["green"], "right": ["blue"]})

CASES = {
    "赤を右側に": [[{"type": "in", "box": "red", "slots": RIGHT_SIDE}]],
    "青を緑の上に": [[{"type": "on", "box": "blue", "under": "green"}]],
    "全部を右側に寄せる": [[{"type": "in", "box": b, "slots": RIGHT_SIDE} for b in P.BOXES]],
    "左側を空ける": [[{"type": "not_at", "box": b, "slot": s} for b in P.BOXES for s in LEFT_SIDE]],
    "3つの指示A(青を緑に→赤を右側に→青をテーブルに)": [
        [{"type": "on", "box": "blue", "under": "green"}],
        [{"type": "in", "box": "red", "slots": RIGHT_SIDE}],
        [{"type": "on_table", "box": "blue"}]],
    "3つの指示B(緑を斜め左に→赤を緑に→青を中央に)": [
        [{"type": "at", "box": "green", "slot": "front_left"}],
        [{"type": "on", "box": "red", "under": "green"}],
        [{"type": "at", "box": "blue", "slot": "center"}]],
}

print("初期配置:", P.describe(init), "\n")
rows = []
for name, stages in CASES.items():
    results = {
        "空間最小・セット": P.plan(init, stages, vol, "volume"),
        "手数最小・セット": P.plan(init, stages, vol, "moves"),
    }
    if len(stages) > 1:
        results["空間最小・逐次"] = P.plan_online(init, stages, vol, "volume")
    for method, r in results.items():
        rows.append((name, method, r))
    vols = {r["volume"] for r in results.values() if r}
    if len(vols) > 1:  # 差が出たケースは手順も表示
        for method, r in results.items():
            P.show(f"{name}:{method}", r)

print(f"{'ケース':40s} {'方法':12s} {'空間cm3':>8s} {'時間s':>6s} {'手数':>4s}")
for name, method, r in rows:
    if r is None:
        print(f"{name:40s} {method:12s} {'実現不可':>8s}")
    else:
        print(f"{name:40s} {method:12s} {r['volume']:8d} {r['time']:6.0f} {r['moves']:4d}")
