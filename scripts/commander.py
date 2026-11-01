#!/usr/bin/env python3
"""日本語の指示で、ロボットアームに積み木を動かさせる(対話型)

使い方:
  python3 commander.py [--reset] [--dry-run] [--objective balanced] [--record 名前]
  指示を入力して Enter。「終了」で終わる。「リセット」で箱と記録を初期状態に戻す。
  --dry-run: ロボットを動かさず、解釈と計画だけを表示する
  --record:  指示ごとに、使った空間を「名前_番号」で記録する(記録ノードが必要)
"""
import argparse
import json
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import planner as P  # noqa: E402
import llm_parser as LP  # noqa: E402
import run_plan as RP  # noqa: E402


def run_plan(extra):
    subprocess.run([sys.executable, os.path.join(HERE, "run_plan.py")] + extra)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--reset", action="store_true")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--objective", default="balanced", choices=["volume", "balanced", "moves"])
    ap.add_argument("--record", default=None)
    args = ap.parse_args()

    parser = LP.Parser()
    print("使うモデル:", parser.model)
    vol = P.Volume(P.load_library(os.path.join(RP.SWEEPS, "library5.json"))) if args.dry_run else None
    st = RP.load_state(args.reset)  # dry-run では、この中だけで状態を進める
    if args.reset and not args.dry_run:
        run_plan(["--reset-state", "[]"])

    history, n = [], 0
    print("指示を入力してください(「終了」で終わる、「リセット」で初期状態に戻す)")
    while True:
        try:
            text = input("\n指示> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            break
        if not text:
            continue
        if text in ("終了", "q", "quit", "exit"):
            break
        if text == "リセット":
            st = RP.load_state(True)
            if not args.dry_run:
                run_plan(["--reset-state", "[]"])
            history.clear()
            print("初期状態に戻しました")
            continue

        cur = st if args.dry_run else RP.load_state(False)
        print("今の配置:", P.describe(cur["stacks"]))
        try:
            d = parser.parse(text, P.describe(cur["stacks"]), history[-5:])
        except Exception as e:  # noqa: BLE001
            print("LLMの呼び出しに失敗しました:", e)
            continue
        if d.get("question"):
            print("ロボット: すみません、", d["question"])
            continue
        if d.get("impossible"):
            print("ロボット: できません。", d["impossible"])
            continue
        stages = d.get("stages") or []
        if not stages:
            print("ロボット:", d.get("reply") or "指示を条件にできませんでした")
            continue
        print("ロボット:", d.get("reply", ""))
        print("解釈:\n" + LP.describe_stages(stages))
        history.append({"text": text, "reply": d.get("reply", "")})

        if args.dry_run:
            r = P.plan(cur["stacks"], stages, vol, args.objective, cur["hand"], cur["hand_mode"],
                       frozenset(cur["used"]), max_moves=2 * len(stages) + 2)
            if r is None:
                print("ロボット: 腕の制約上、実現できる手順が見つかりませんでした")
                continue
            P.show("計画", r)
            cur.update({"stacks": r["final"], "hand": r["hand"], "hand_mode": r["hand_mode"],
                        "used": sorted(r["parts"])})
        else:
            n += 1
            extra = ["--record", f"{args.record}_{n}"] if args.record else []
            run_plan(["--batch", "--objective", args.objective, "--tag", "commander"]
                     + extra + [json.dumps(stages)])


if __name__ == "__main__":
    main()
