# 日本語の指示で、使う空間を最小にしながら積み木を動かすロボット

日本語で話しかけると、ロボットアーム CRANE+ V2 が指示を理解し、**アームが通る空間をできるだけ小さく保ちながら**積み木を動かすシステムです。Gazebo(Ignition Fortress)上で動作します。

- 言葉の理解は LLM(Google Gemini)、物理的に正しく効率の良い手順の決定は探索アルゴリズム、と役割を分けています
- メーカー提供のサンプルと同じ動かし方(1手ごとに home 姿勢へ戻る)と比べて、アームが通る空間を約17%、実行時間を約22%削減しました(Gazebo での比較実験、3組の指示の平均)

## 構成

| ファイル | 役割 |
| --- | --- |
| `src/pick_place_server.cpp` | 実行ノード。指令を受けて箱を掴んで置く(比較用の naive モードを含む) |
| `scripts/llm_parser.py` | Gemini で日本語の指示を条件(JSON)に変換する |
| `scripts/planner.py` | 計画器。条件を満たし、使う空間と時間が最良の手順を探索する |
| `scripts/commander.py` | 対話型のプログラム。指示の入力から解釈・計画・実行まで |
| `scripts/run_plan.py` | 計画を実行ノードに送り、箱の位置補正と結果の記録を行う |
| `scripts/poses.py` | Gazebo から箱の位置を読み、置き場所からのずれを求める |
| `scripts/swept_volume_recorder.py` | アームが通った空間を記録し、RViz に表示する |
| `scripts/build_library5.py` | 動作の部品ごとの空間を測り、ライブラリを作る |
| `scripts/eval_random.py`, `experiment.py`, `analyze.py` | 評価と集計 |
| `worlds/three_cubes.sdf`, `worlds/empty_table.sdf` | 3色の箱を置いたワールド、箱のないワールド |
| `data/library5.json` | 実測した部品ごとの空間のライブラリ(`~/sweeps/` にコピーして使う) |

## 動作環境

- Docker イメージ `tiryoh/ros2-desktop-vnc:humble`(Apple Silicon の Mac では arm64 版が動作)
- ROS 2 Humble、Gazebo(Ignition Fortress)、MoveIt 2
- [rt-net/crane_plus](https://github.com/rt-net/crane_plus)(humble ブランチ)
- Python パッケージ `google-genai`、Gemini の API キー

## 準備

```bash
# crane_plus と本パッケージを同じワークスペースに置いてビルド
cd ~/ros2_ws/src
git clone -b humble https://github.com/rt-net/crane_plus.git
git clone <このリポジトリのURL> llm_arm
cd ~/ros2_ws
rosdep install -r -y -i --from-paths src --ignore-src
colcon build --symlink-install
source install/setup.bash

# 部品ライブラリを置く
mkdir -p ~/sweeps && cp ~/ros2_ws/src/llm_arm/data/library5.json ~/sweeps/

# LLM の設定(API キーはコードに書かず、環境変数で渡す)
pip3 install --user google-genai
export GEMINI_API_KEY="(自分のAPIキー)"
export GEMINI_MODEL="(使うモデル名)"
```

## 実行

ターミナルを4つ使います。

```bash
# 1: Gazebo(3色の箱のワールド)
ros2 launch crane_plus_gazebo crane_plus_with_table.launch.py \
  world_name:=$HOME/ros2_ws/src/llm_arm/worlds/three_cubes.sdf

# 2: 実行ノード
ros2 launch llm_arm server.launch.py example:='pick_place_server' use_sim_time:='true'

# 3: 空間の記録ノード(RViz で /swept_volume_markers の Marker を追加すると表示される)
ros2 run llm_arm swept_volume_recorder.py --ros-args -p use_sim_time:=true

# 4: 日本語で指示する
cd ~/ros2_ws/src/llm_arm/scripts
python3 commander.py --reset
```

指示の例: 「赤を右のほうに置いて」「それを青の上に積んで」「緑を斜め左に置いてから、その上に青を載せて」。`--dry-run` を付けると、ロボットを動かさずに解釈と計画だけを確認できます。

## 比較実験

```bash
python3 experiment.py   # 3組の指示 × 5つの方法を Gazebo で実行
python3 analyze.py exp_ # 集計
python3 eval_random.py 100 1   # ランダムな指示100組で計画器を評価(ロボット不要)
```

## クレジット

実行ノードは RT Corporation の [crane_plus_examples](https://github.com/rt-net/crane_plus) の `pick_and_place.cpp`(Apache License 2.0)を参考にしています。
