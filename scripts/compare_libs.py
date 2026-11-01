#!/usr/bin/env python3
"""計算で作った仮ライブラリ(library5_synth)と、実測ライブラリ(library5)を部品ごとに比べる"""
import json
import os
from collections import defaultdict

d = os.path.expanduser("~/sweeps")
synth = {k: set(v) for k, v in json.load(open(os.path.join(d, "library5_synth.json"))).items()}
meas = {k: set(v) for k, v in json.load(open(os.path.join(d, "library5.json"))).items()}


def group(k):
    kind = k.split(":")[0]
    mode = "高" if k.endswith(":high") or ":high:" in k else ("低" if "low" in k else "-")
    diag = "斜めを含む" if "front" in k else "左右中央のみ"
    return f"{kind:6s} {mode} {diag}"


rows, stats = [], defaultdict(list)
for k in sorted(set(synth) & set(meas)):
    if k == "free":
        continue
    a, b = meas[k], synth[k]
    iou = len(a & b) / len(a | b) if a | b else 1.0
    ratio = len(b) / len(a) if a else float("nan")
    rows.append((k, len(a), len(b), iou, ratio))
    stats[group(k)].append((iou, ratio))

print(f"{'部品':32s} {'実測':>6s} {'計算':>6s} {'一致度':>6s} {'計算/実測':>8s}")
for k, a, b, iou, ratio in rows:
    print(f"{k:32s} {a:6d} {b:6d} {iou:6.2f} {ratio:8.2f}")
print("\n===== 種類ごとの平均 =====")
print(f"{'種類':24s} {'一致度':>6s} {'計算/実測':>8s} {'個数':>4s}")
for g in sorted(stats):
    v = stats[g]
    print(f"{g:24s} {sum(x for x, _ in v) / len(v):6.2f} {sum(y for _, y in v) / len(v):8.2f} {len(v):4d}")
missing = sorted(set(synth) ^ set(meas))
if missing:
    print("\n片方にしかない部品:", missing)
