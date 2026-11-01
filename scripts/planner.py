#!/usr/bin/env python3
"""積み木の計画器(掃引空間の最小化版、置き場所の数を変更可能)

目標条件を順に満たす手順のうち、目的(objective)が最小になるものを探す。
  objective="volume": アームが通る空間(部品ライブラリの和集合の体積)を最小化
  objective="moves" : 動かす回数を最小化(比較用)
和集合なので、一度使った部品をもう一度使っても体積は増えない。

状態 = (各置き場所の山, 手の場所, 手の姿勢, 何番目の指示まで終えたか, 使った部品の集合)
探索 = ダイクストラ法(コストは (主目的, 副目的, ...) の組で比較)
"""
import heapq
import itertools
import json
import math
import os

import numpy as np

# ---- 環境と腕の制約(実験で確認した値) ----
BOX = 0.03
GRASP = 0.025
MARGIN = 0.008
TOPDOWN_MAX = 0.06
HIGH_TRAVEL = 0.10
LOW_TRAVEL = MARGIN + GRASP
MAX_STACK = 2
ALPHA = 15.0  # balanced で、時間1秒を何 cm3 とみなすか
TIME = {("low", "low"): 13.0, ("high", "low"): 16.0,
        ("low", "high"): 16.0, ("high", "high"): 19.5}

SLOT_RADIUS = 0.1
ALL_SLOT_ANGLES = {"left": 90, "front_left": 45, "center": 0, "front_right": -45, "right": -90}
BOXES = ("red", "green", "blue")
JP = {"red": "赤", "green": "緑", "blue": "青",
      "left": "左", "front_left": "斜め左", "center": "中央", "front_right": "斜め右",
      "right": "右", "low": "低", "high": "高"}
LIB_PATH = os.path.expanduser("~/sweeps/library.json")

SLOT_ORDER = ()
SLOTS = {}


def configure(names):
    """使う置き場所を設定する(左から順に並べる)"""
    global SLOT_ORDER, SLOTS
    SLOT_ORDER = tuple(sorted(names, key=lambda n: -ALL_SLOT_ANGLES[n]))
    SLOTS = {n: (round(SLOT_RADIUS * math.cos(math.radians(ALL_SLOT_ANGLES[n])), 4),
                 round(SLOT_RADIUS * math.sin(math.radians(ALL_SLOT_ANGLES[n])), 4))
             for n in SLOT_ORDER}


configure(["left", "center", "right"])


def load_library(path=LIB_PATH):
    with open(path) as f:
        return {k: frozenset(v) for k, v in json.load(f).items()}


def pair(a, b):
    return "-".join(sorted((a, b)))


def slots_on_path(a, b):
    """置き場所 a から b へ根元を回すとき、上を通る置き場所(正面側を通る)"""
    lo, hi = sorted((ALL_SLOT_ANGLES[a], ALL_SLOT_ANGLES[b]))
    return {s for s in SLOT_ORDER if lo <= ALL_SLOT_ANGLES[s] <= hi}


def need_height(heights):
    return max(heights, default=0) * BOX + MARGIN + GRASP


def to_key(stacks):
    return tuple(tuple(stacks[s]) for s in SLOT_ORDER)


def to_dict(key):
    return {s: list(key[i]) for i, s in enumerate(SLOT_ORDER)}


def empty_stacks():
    return {s: [] for s in SLOT_ORDER}


def positions(stacks):
    return {box: (slot, level) for slot, st in stacks.items() for level, box in enumerate(st)}


# ---- 目標条件 ----
def validate(c):
    t = c.get("type")
    if t not in ("at", "not_at", "in", "on", "on_table", "clear", "stack"):
        raise ValueError(f"不明な条件: {t}")
    for k in ("box", "under"):
        if k in c and c[k] not in BOXES:
            raise ValueError(f"不明な箱: {c[k]}")
    if "slot" in c and c["slot"] not in SLOTS:
        raise ValueError(f"不明な置き場所: {c['slot']}")
    if t == "in" and any(s not in SLOTS for s in c["slots"]):
        raise ValueError(f"不明な置き場所: {c['slots']}")
    if t == "stack" and any(b not in BOXES for b in c["boxes"]):
        raise ValueError(f"不明な箱: {c['boxes']}")


def satisfied(stacks, c):
    pos = positions(stacks)
    t = c["type"]
    if t == "at":
        return pos[c["box"]][0] == c["slot"]
    if t == "not_at":
        return pos[c["box"]][0] != c["slot"]
    if t == "in":        # 箱が、指定した置き場所のどれかにある
        return pos[c["box"]][0] in c["slots"]
    if t == "on":
        (s1, l1), (s2, l2) = pos[c["box"]], pos[c["under"]]
        return s1 == s2 and l1 == l2 + 1
    if t == "on_table":
        return pos[c["box"]][1] == 0
    if t == "clear":
        s, l = pos[c["box"]]
        return l == len(stacks[s]) - 1
    if t == "stack":
        return list(stacks[c["slot"]]) == list(c["boxes"])
    return False


# ---- 1手の実行方法の候補 ----
def move_options(stacks, hand, hand_mode, src, dst):
    if src == dst or not stacks[src] or len(stacks[dst]) >= MAX_STACK:
        return []
    box = stacks[src][-1]
    after = {s: list(v) for s, v in stacks.items()}
    after[src].pop()
    lp, lq = len(stacks[src]) - 1, len(after[dst])
    pz, qz = lp * BOX + GRASP, lq * BOX + GRASP

    if hand is None:
        app_h = [len(stacks[src])]
    elif hand == src:
        app_h = []
    else:
        app_h = [len(stacks[s]) for s in slots_on_path(hand, src)]
    car_h = [len(after[s]) for s in slots_on_path(src, dst)]

    app_modes = [m for m in ("low", "high")
                 if (m == "high" and need_height(app_h) <= HIGH_TRAVEL + 1e-9)
                 or (m == "low" and lp == 0 and need_height(app_h) <= TOPDOWN_MAX + 1e-9)]
    car_modes = [m for m in ("low", "high")
                 if (m == "high" and need_height(car_h) <= HIGH_TRAVEL + 1e-9)
                 or (m == "low" and lp == 0 and lq == 0
                     and need_height(car_h) <= TOPDOWN_MAX + 1e-9)]

    after[dst].append(box)
    travel = {"low": LOW_TRAVEL, "high": HIGH_TRAVEL}
    options = []
    for ma in app_modes:
        for mc in car_modes:
            parts = []
            if hand is None:
                parts.append("free")
            else:
                if ma != hand_mode:
                    parts.append(f"adjust:{hand}")
                parts.append(f"arc:{pair(hand, src)}:{ma}")
            parts += [f"vert:{src}:{ma}:{lp}", f"vert:{src}:{mc}:{lp}",
                      f"arc:{pair(src, dst)}:{mc}", f"vert:{dst}:{mc}:{lq}"]
            px, py = SLOTS[src]
            qx, qy = SLOTS[dst]
            cmd = [px, py, round(pz, 4), qx, qy, round(qz, 4), travel[mc], travel[ma]]
            step = {"box": box, "src": src, "dst": dst, "ma": ma, "mc": mc,
                    "level_from": lp, "level_to": lq, "cmd": cmd}
            options.append(({s: list(v) for s, v in after.items()}, parts, TIME[(ma, mc)], step))
    return options


# ---- 探索 ----
class Volume:
    """部品の集合 → 和集合の体積。各部品をビット列(1ボクセル=1ビット)で持ち、和集合はビットOR"""
    def __init__(self, lib):
        ids = sorted(set().union(*lib.values()))
        index = {v: i for i, v in enumerate(ids)}
        self.mask = {}
        for k, vs in lib.items():
            bits = np.zeros(len(ids), dtype=bool)
            bits[[index[v] for v in vs]] = True
            self.mask[k] = int.from_bytes(np.packbits(bits, bitorder="little").tobytes(), "little")

    def of(self, parts):
        m = 0
        for p in parts:
            if p not in self.mask:
                raise KeyError(f"ライブラリにない部品: {p}")
            m |= self.mask[p]
        return m

    def __call__(self, parts):
        return self.of(parts).bit_count()


def cost_tuple(objective, vol, time, moves):
    if objective == "volume":
        return (vol, time, moves)
    if objective == "moves":
        return (moves, time, vol)
    if objective == "balanced":  # 空間 + 時間(1秒 = ALPHA cm3 とみなす)
        return (vol + ALPHA * time, vol, moves)
    raise ValueError(objective)


def plan(initial, stages, vol, objective="volume", hand=None, hand_mode="high",
         used=frozenset(), max_moves=5, keep=4):
    """stages: 指示ごとの条件リストの列。順に満たす手順を返す(解けなければ None)
    keep: 同じ配置・手の状態ごとに残す候補の数の上限(探索を軽くするための近似)"""
    for st in stages:
        for c in st:
            validate(c)
    used = frozenset(used)
    nodes = [{"parent": None, "events": [], "mask": vol.of(used), "time": 0.0, "moves": 0}]
    seen = {}

    def admit(state, cost):
        lst = seen.setdefault(state[:4], [])
        for parts, c in lst:
            if parts <= state[4] and c <= cost:  # 部品が多いのに得でない → 捨てる
                return False
        if len(lst) >= keep and cost >= max(c for _, c in lst):
            return False
        lst.append((state[4], cost))
        return True

    counter = itertools.count()
    start = (to_key(initial), hand, hand_mode, 0, used)
    c0 = cost_tuple(objective, nodes[0]["mask"].bit_count(), 0.0, 0)
    admit(start, c0)
    queue = [(c0, next(counter), 0, start)]
    while queue:
        cost, _, i, state = heapq.heappop(queue)
        key, h, hm, stage, parts = state
        stacks = to_dict(key)
        node = nodes[i]
        markers = []
        while stage < len(stages) and all(satisfied(stacks, c) for c in stages[stage]):
            markers.append({"stage_done": stage})
            stage += 1
        if stage == len(stages):
            events, j = markers, i
            while j is not None:
                events = nodes[j]["events"] + events
                j = nodes[j]["parent"]
            return {"steps": events, "parts": parts, "volume": node["mask"].bit_count(),
                    "time": node["time"], "moves": node["moves"], "final": stacks,
                    "hand": h, "hand_mode": hm}
        if node["moves"] >= max_moves:
            continue
        for src in SLOT_ORDER:
            for dst in SLOT_ORDER:
                for after, new_parts, t, step in move_options(stacks, h, hm, src, dst):
                    m2 = node["mask"] | vol.of(new_parts)
                    t2, mv2 = node["time"] + t, node["moves"] + 1
                    c2 = cost_tuple(objective, m2.bit_count(), t2, mv2)
                    s2 = (to_key(after), dst, step["mc"], stage, parts | frozenset(new_parts))
                    if not admit(s2, c2):
                        continue
                    nodes.append({"parent": i, "events": markers + [step],
                                  "mask": m2, "time": t2, "moves": mv2})
                    heapq.heappush(queue, (c2, next(counter), len(nodes) - 1, s2))
    return None


def plan_online(initial, stages, vol, objective="volume", hand=None, hand_mode="high"):
    """指示を1つずつ、そのつど最適化する(逐次)。使った部品は次の指示に引き継ぐ"""
    stacks, used, steps = initial, frozenset(), []
    total_time, total_moves = 0.0, 0
    for i, st in enumerate(stages):
        r = plan(stacks, [st], vol, objective, hand, hand_mode, used)
        if r is None:
            return None
        steps += [s if "stage_done" not in s else {"stage_done": i} for s in r["steps"]]
        stacks, used, hand, hand_mode = r["final"], r["parts"], r["hand"], r["hand_mode"]
        total_time += r["time"]
        total_moves += r["moves"]
    return {"steps": steps, "parts": used, "volume": vol(used), "time": total_time,
            "moves": total_moves, "final": stacks, "hand": hand, "hand_mode": hand_mode}


# ---- 表示 ----
def describe(stacks):
    return " / ".join(f"{JP[s]}:[{','.join(JP[b] for b in stacks[s])}]" for s in SLOT_ORDER)


def show(title, result):
    print(f"■ {title}")
    if result is None:
        print("  → 腕の制約内では実現できません\n")
        return
    i = 0
    for s in result["steps"]:
        if "stage_done" in s:
            print(f"    -- 指示{s['stage_done'] + 1} 完了 --")
            continue
        i += 1
        print(f"  {i}. {JP[s['box']]}を {JP[s['src']]} → {JP[s['dst']]}({s['level_to'] + 1}段目)"
              f"  近づく:{JP[s['ma']]} 運ぶ:{JP[s['mc']]}")
    print(f"  最終配置: {describe(result['final'])}")
    print(f"  予測: 空間 {result['volume']} cm3 / 時間 {result['time']:.0f} 秒 / "
          f"{result['moves']} 手\n")


if __name__ == "__main__":
    vol = Volume(load_library())
    init = {"left": ["red"], "center": ["green"], "right": ["blue"]}
    print("初期配置:", describe(init), "\n")
    show("赤を右に(空間最小)", plan(init, [[{"type": "at", "box": "red", "slot": "right"}]], vol))
