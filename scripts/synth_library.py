#!/usr/bin/env python3
"""回転対称性を使って、測定済みの部品から、5か所分の部品ライブラリを計算で作る

- 上げ下げ・姿勢の切り替え: 正面(center)で測った部品を、置き場所の角度だけ根元の軸周りに回転
- 置き場所の間の回転: 回転中の姿勢(正面で測った arc:center-center)を 1度ずつ回して重ねる
正しさの確認として、測定済みの左・右の部品と、計算で作った左・右の部品を比べる。
"""
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import planner  # noqa: E402

VOXEL = 0.01
OFFSET = 512
SUB = np.array([(dx, dy, dz) for dx in (0.25, 0.75) for dy in (0.25, 0.75) for dz in (0.25, 0.75)])


def decode(keys):
    keys = np.asarray(sorted(keys), dtype=np.int64)
    return np.stack([(keys >> 20) & 1023, (keys >> 10) & 1023, keys & 1023], axis=1) - OFFSET


def encode(idx):
    i, j, k = (idx + OFFSET).T
    return (i.astype(np.int64) << 20) | (j.astype(np.int64) << 10) | k.astype(np.int64)


def rotate(keys, deg):
    """ボクセルの集合を、根元の軸(z軸)周りに deg 度回す(1つのボクセルを8点で代表させて隙間を防ぐ)"""
    if not keys:
        return set()
    idx = decode(keys)
    pts = ((idx[:, None, :] + SUB[None, :, :]) * VOXEL).reshape(-1, 3)
    t = np.radians(deg)
    x = pts[:, 0] * np.cos(t) - pts[:, 1] * np.sin(t)
    y = pts[:, 0] * np.sin(t) + pts[:, 1] * np.cos(t)
    new = np.floor(np.stack([x, y, pts[:, 2]], axis=1) / VOXEL).astype(np.int64)
    return set(np.unique(encode(new)).tolist())


def sweep(keys, a, b):
    """姿勢 keys を、角度 a から b まで 1度ずつ回して重ねる"""
    lo, hi = sorted((a, b))
    out = set()
    for d in np.arange(lo, hi + 0.5, 1.0):
        out |= rotate(keys, d)
    return out


def slice_arc(keys, lo, hi, margin=0.015):
    """実測の回転部品から、角度 lo〜hi の範囲だけを切り出す
    (根元から r の位置では、腕の太さ margin を角度に直した分だけ範囲を広げる。根元付近は常に残す)"""
    idx = decode(keys)
    c = (idx + 0.5) * VOXEL
    r = np.hypot(c[:, 0], c[:, 1])
    az = np.degrees(np.arctan2(c[:, 1], c[:, 0]))
    w = np.degrees(np.arctan2(margin, np.maximum(r, 1e-6)))
    keep = (r < 0.03) | ((az >= lo - w) & (az <= hi + w))
    return set(encode(idx[keep]).tolist())


def build(measured, names):
    ang = planner.ALL_SLOT_ANGLES
    lib = {"free": set(measured["free"])}
    for n in names:
        for key, val in measured.items():
            if key.startswith("vert:center:"):
                lib[key.replace("vert:center:", f"vert:{n}:")] = rotate(val, ang[n])
        lib[f"adjust:{n}"] = rotate(measured["adjust:center"], ang[n])
    for m in ("low", "high"):
        full = measured[f"arc:left-right:{m}"]          # 実測の 180 度分の回転
        still = measured[f"arc:center-center:{m}"]      # 掴むときの小さな回転(3度)
        for a in names:
            for b in names:
                if a < b or a == b:
                    if a == b:
                        lib[f"arc:{a}-{a}:{m}"] = rotate(still, ang[a])
                    else:
                        lo, hi = sorted((ang[a], ang[b]))
                        lib[f"arc:{planner.pair(a, b)}:{m}"] = slice_arc(full, lo, hi)
    return lib


def compare(measured, synth):
    print(f"{'部品':28s} {'実測':>6s} {'計算':>6s} {'一致度':>6s}")
    keys = [k for k in sorted(measured) if k in synth and k != "free"
            and ("left" in k or "right" in k) and "front" not in k]
    ious = []
    for k in keys:
        a, b = set(measured[k]), synth[k]
        iou = len(a & b) / len(a | b)
        ious.append(iou)
        print(f"{k:28s} {len(a):6d} {len(b):6d} {iou:6.2f}")
    print(f"平均一致度(重なり / 合わせた範囲): {np.mean(ious):.2f}\n")


if __name__ == "__main__":
    src = os.path.expanduser("~/sweeps/library.json")
    with open(src) as f:
        measured = {k: set(v) for k, v in json.load(f).items()}
    names = list(planner.ALL_SLOT_ANGLES)
    synth = build(measured, names)
    compare(measured, synth)
    out = os.path.expanduser("~/sweeps/library5_synth.json")
    with open(out, "w") as f:
        json.dump({k: sorted(v) for k, v in synth.items()}, f)
    print(f"保存しました: {out} ({len(synth)} 部品)")
