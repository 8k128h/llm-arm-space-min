#!/usr/bin/env python3
"""results.jsonl と、記録ノードが保存した実測の掃引空間(~/sweeps/名前.npy)を集計する

使い方: python3 analyze.py [記録名の先頭]   (例: python3 analyze.py exp_)
"""
import json
import os
import sys
from collections import defaultdict

import numpy as np

SWEEPS = os.path.expanduser("~/sweeps")
prefix = sys.argv[1] if len(sys.argv) > 1 else ""

latest = {}
with open(os.path.join(SWEEPS, "results.jsonl")) as f:
    for line in f:
        r = json.loads(line)
        if r.get("record") and r["record"].startswith(prefix):
            latest[r["record"]] = r  # 同じ名前は最後の結果を使う


def method_of(r):
    if r["naive"]:
        return "naive"
    return f"{r['objective']}・{'セット' if r['batch'] else '逐次'}"


rows = []
for name, r in sorted(latest.items()):
    path = os.path.join(SWEEPS, name + ".npy")
    measured = len(np.load(path)) if os.path.exists(path) else None
    rows.append((r["tag"], method_of(r), measured, r["predicted"], r["exec_time"], r["moves"], r["ok"]))

print(f"{'組':4s} {'方法':16s} {'実測cm3':>8s} {'予測cm3':>8s} {'予測誤差':>8s} {'時間s':>6s} {'手数':>4s} 成否")
for tag, m, meas, pred, t, mv, ok in rows:
    err = f"{100 * (pred - meas) / meas:+.1f}%" if meas and pred else "-"
    print(f"{tag:4s} {m:16s} {meas if meas else '-':>8} {pred if pred else '-':>8} {err:>8s} "
          f"{t:6.1f} {mv:4d} {'ok' if ok else '失敗'}")

# naive を基準にした比較(同じ組どうし)
base = {tag: (meas, t) for tag, m, meas, pred, t, mv, ok in rows if m == "naive" and meas}
summary = defaultdict(list)
for tag, m, meas, pred, t, mv, ok in rows:
    if tag in base and meas and ok:
        bv, bt = base[tag]
        summary[m].append((meas / bv - 1, t / bt - 1))
if summary:
    print("\n===== naive を基準にした平均 =====")
    print(f"{'方法':16s} {'空間':>8s} {'時間':>8s} {'組の数':>6s}")
    for m, v in sorted(summary.items()):
        print(f"{m:16s} {100 * sum(a for a, _ in v) / len(v):+7.1f}% "
              f"{100 * sum(b for _, b in v) / len(v):+7.1f}% {len(v):6d}")
