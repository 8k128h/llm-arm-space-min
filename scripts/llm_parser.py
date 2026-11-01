#!/usr/bin/env python3
"""日本語の指示を、計画器が読める条件(JSON)に変換する(Gemini API を使用)

APIキーは環境変数 GEMINI_API_KEY から読む(コードには書かない)。
モデルは環境変数 GEMINI_MODEL で指定できる。未指定なら、使えるモデルから flash 系を自動で選ぶ。

単独で実行すると、指示を変換した結果を表示する:
  python3 llm_parser.py "赤を青の上に積んで"
  python3 llm_parser.py --list-models
"""
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import planner as P  # noqa: E402

P.configure(["left", "front_left", "center", "front_right", "right"])

SYSTEM = """あなたは、ロボットアームへの日本語の指示を、計画器が読める条件(JSON)に変換する係です。
手順や動かし方は計画器が決めるので、あなたは「最終的にどうなっていればよいか」だけを書きます。

## 環境
- 箱: red(赤), green(緑), blue(青)。一辺3cmの立方体
- 置き場所(ロボットから見て左から順に): left(左), front_left(斜め左), center(中央・正面),
  front_right(斜め右), right(右)
- 「右側」「右のほう」は ["front_right", "right"]、「左側」「左のほう」は ["left", "front_left"]
- 箱は2段まで積める。3段以上は積めない

## 出力するJSON(これだけを出力する)
{"stages": [...], "reply": "利用者への短い返答(日本語)", "question": null または 聞き返す内容,
 "impossible": null または 実現できない理由}

stages は「段階」のリスト。各段階は、その時点で同時に満たすべき条件のリスト。
「AしてからB」「Aして、そのあとB」のように順番がある指示は、段階を分ける。
順番に意味がない指示は、1つの段階にまとめる。

条件の種類(box は red/green/blue、slot は上の5つの英語名):
- {"type": "at", "box": B, "slot": S}            箱Bが置き場所Sにある(何段目かは問わない)
- {"type": "not_at", "box": B, "slot": S}        箱Bが置き場所Sにない
- {"type": "in", "box": B, "slots": [S, ...]}    箱Bが、いずれかの置き場所にある
- {"type": "on", "box": B, "under": U}           箱Bが箱Uのすぐ上に積まれている
- {"type": "on_table", "box": B}                 箱Bがテーブルに直接置かれている(積まれていない)
- {"type": "clear", "box": B}                    箱Bの上に何も載っていない
- {"type": "stack", "slot": S, "boxes": [下から順]}  置き場所Sの山が、ちょうどこの並び

## ルール
- 指示に出てこない箱や条件は、勝手に加えない(どこに置くかは、指示の範囲で計画器が最適に選ぶ)
- 「それ」「さっきの箱」などは、会話の履歴から判断する
- 意味を1つに決められないときは、stages を [] にして question で聞き返す
- 3段に積むなど、ルール上できないときは、stages を [] にして impossible に理由を書く
- reply には、どう解釈したかを一言で書く

## 例
指示: 赤を青の上に積んで
{"stages": [[{"type": "on", "box": "red", "under": "blue"}]], "reply": "赤を青の上に積みます",
 "question": null, "impossible": null}

指示: 緑を右のほうに置いてから、その上に赤を載せて
{"stages": [[{"type": "in", "box": "green", "slots": ["front_right", "right"]}],
            [{"type": "on", "box": "red", "under": "green"}]],
 "reply": "緑を右側に置き、その上に赤を積みます", "question": null, "impossible": null}

指示: 右側を空けて
{"stages": [[{"type": "not_at", "box": "red", "slot": "front_right"},
             {"type": "not_at", "box": "red", "slot": "right"},
             {"type": "not_at", "box": "green", "slot": "front_right"},
             {"type": "not_at", "box": "green", "slot": "right"},
             {"type": "not_at", "box": "blue", "slot": "front_right"},
             {"type": "not_at", "box": "blue", "slot": "right"}]],
 "reply": "右側の2か所から箱をどけます", "question": null, "impossible": null}

指示: 全部まとめて1か所に積んで
{"stages": [], "reply": "", "question": null,
 "impossible": "箱は2段までしか積めないので、3つを1か所にまとめることはできません"}
"""

JP_TYPE = {
    "at": lambda c: f"{P.JP[c['box']]}が{P.JP[c['slot']]}にある",
    "not_at": lambda c: f"{P.JP[c['box']]}が{P.JP[c['slot']]}にない",
    "in": lambda c: f"{P.JP[c['box']]}が{'・'.join(P.JP[s] for s in c['slots'])}のどれかにある",
    "on": lambda c: f"{P.JP[c['box']]}が{P.JP[c['under']]}のすぐ上にある",
    "on_table": lambda c: f"{P.JP[c['box']]}がテーブルに直接置かれている",
    "clear": lambda c: f"{P.JP[c['box']]}の上に何もない",
    "stack": lambda c: f"{P.JP[c['slot']]}の山が下から{'・'.join(P.JP[b] for b in c['boxes'])}",
}


def describe_stages(stages):
    return "\n".join(f"  段階{i + 1}: " + " かつ ".join(JP_TYPE[c["type"]](c) for c in st)
                     for i, st in enumerate(stages))


class Parser:
    def __init__(self):
        from google import genai
        from google.genai import types
        key = os.environ.get("GEMINI_API_KEY")
        if not key:
            raise RuntimeError("環境変数 GEMINI_API_KEY が設定されていません")
        self.types = types
        try:  # ライブラリ任せの自動再試行(長時間待つことがある)をやめ、30秒で打ち切る
            opts = types.HttpOptions(timeout=30000,
                                     retry_options=types.HttpRetryOptions(attempts=1))
            self.client = genai.Client(api_key=key, http_options=opts)
        except (AttributeError, TypeError):
            self.client = genai.Client(api_key=key, http_options=types.HttpOptions(timeout=30000))
        self.model = os.environ.get("GEMINI_MODEL") or self.pick_model()

    def models(self):
        names = []
        for m in self.client.models.list():
            actions = getattr(m, "supported_actions", None) or []
            if not actions or "generateContent" in actions:
                names.append(m.name.replace("models/", ""))
        return names

    def candidates(self):
        """flash 系のモデルを、安定版・新しい順に並べる(lite は最後の候補)"""
        names = [n for n in self.models() if "flash" in n
                 and not any(w in n for w in ("image", "tts", "live", "audio", "thinking"))]

        def rank(n):
            return ("lite" in n, any(w in n for w in ("preview", "exp")), [-ord(ch) for ch in n])
        return sorted(names, key=rank)

    def candidates(self):
        """flash 系のモデルを、安定版・新しい順に並べる(lite は最後の候補)"""
        names = [n for n in self.models() if "flash" in n
                 and not any(w in n for w in ("image", "tts", "live", "audio", "thinking"))]

        def rank(n):
            return ("lite" in n, any(w in n for w in ("preview", "exp")), [-ord(ch) for ch in n])
        return sorted(names, key=rank)

    def pick_model(self):
        cands = self.candidates()
        if not cands:
            raise RuntimeError("使える flash 系のモデルが見つかりません。GEMINI_MODEL で指定してください")
        self.fallbacks = cands[1:]
        return cands[0]

    def probe(self):
        """各モデルに短い問い合わせを1回ずつ送り、使えるかどうかを調べる"""
        for n in self.candidates():
            print(f"  確認中 … {n}", flush=True)
            try:
                self.client.models.generate_content(model=n, contents="OK とだけ答えて")
                print(f"  使える      {n}")
            except Exception as e:  # noqa: BLE001
                code = getattr(e, "code", "")
                print(f"  使えない({code}) {n}")

    def ask(self, prompt):
        while True:
            try:
                resp = self.client.models.generate_content(
                    model=self.model, contents=prompt,
                    config=self.types.GenerateContentConfig(
                        system_instruction=SYSTEM, response_mime_type="application/json",
                        temperature=0.0))
                return json.loads(resp.text)
            except Exception as e:  # noqa: BLE001
                code = getattr(e, "code", None)
                rest = getattr(self, "fallbacks", [])
                if code in (404, 429) and rest:  # 使えない・上限 → 次の候補へ
                    print(f"  ({self.model} は使えません [{code}]。{rest[0]} に切り替えます)")
                    self.model = rest.pop(0)
                    continue
                raise

    def parse(self, text, state_desc="", history=()):
        hist = "\n".join(f"- 指示: {h['text']} → 解釈: {h['reply']}" for h in history) or "(なし)"
        prompt = f"今の配置: {state_desc}\n会話の履歴:\n{hist}\n\n指示: {text}"
        for attempt in range(2):
            data = self.ask(prompt)
            try:
                for st in data.get("stages", []):
                    for c in st:
                        P.validate(c)
                return data
            except (ValueError, KeyError, TypeError) as e:  # 形式の誤りは、理由を伝えて1回だけやり直す
                prompt += f"\n\n(前回の出力は条件の形式が正しくありませんでした: {e}。JSONを直して出力してください)"
        raise ValueError("LLMの出力を条件に変換できませんでした")


if __name__ == "__main__":
    p = Parser()
    if len(sys.argv) > 1 and sys.argv[1] == "--list-models":
        print("\n".join(p.models()))
        sys.exit()
    if len(sys.argv) > 1 and sys.argv[1] == "--probe":
        p.probe()
        sys.exit()
    if len(sys.argv) > 1 and sys.argv[1] == "--probe":
        p.probe()
        sys.exit()
    print("使うモデル:", p.model)
    init = P.empty_stacks()
    init.update({"left": ["red"], "center": ["green"], "right": ["blue"]})
    for text in sys.argv[1:]:
        d = p.parse(text, P.describe(init))
        print(f"\n指示: {text}")
        print("返答:", d.get("reply"))
        if d.get("question"):
            print("聞き返し:", d["question"])
        if d.get("impossible"):
            print("実現できない:", d["impossible"])
        if d.get("stages"):
            print(describe_stages(d["stages"]))
            print(json.dumps(d["stages"], ensure_ascii=False))
