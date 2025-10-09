# [MODULE]: controllers
[PURPOSE]:
  経路追従・目的地点モード・適応制御などの上位制御アルゴリズムをまとめる階層。
  差動二輪ロボットの制御を対象とし、速度・ステアリング角を生成する責務を持つ。

[RESPONSIBILITY]:
  経路情報・車両姿勢から最適な制御入力 (v, δ) を生成。
  以下の制御方式を提供：
  - 経路追従モード（Path Following）
  - 目的地点モード（Pose Regulation, Dubins Path）
  - 適応最適化モード（PSO / 強化学習ベース）

[DEPENDS_ON]:
  - models.physics
  - models.geometry
  - trajectory.utils

[USED_BY]:
  - sim.simulation
  - sim.vehicle_manager

---

## 📂 ファイル一覧

| ファイル名 | 概要 | 主な関数・クラス |
|:--|:--|:--|
| `path_follow.py` | 経路追従モードの制御。CTE・θerr・ωerrを用いた操舵制御を実装。 | `compute_control()`, `update_errors()`, `follow_path()` |
| `pose_regulation.py` | 目的地点モード。Lyapunov関数や磁気ベクトル場を用いて姿勢収束。 | `pose_control()`, `compute_distance_error()`, `compute_heading_error()` |
| `pso_adapt.py` | PSOベースの適応制御。補正係数の最適化を担当。 | `optimize_params()`, `evaluate_fitness()`, `update_particles()` |

---

## ⚙️ インターフェース仕様

| 入力 | 出力 | 補足 |
|:--|:--|:--|
| `state = (x, y, θ)` | 車両の現在状態 | - |
| `path_data` | 経路上の点列、局所曲率、姿勢情報 | - |
| `mode` | 制御モード識別子（"path_follow", "pose_reg" など） | - |
| **出力:** `(v, δ)` | 並進速度とステアリング角 | 物理モデルまたは実機へ渡す |

---

## 🧩 関数リスト（全体要約）

| 関数 / クラス名 | 入出力 | 概要 | 所属ファイル |
|:--|:--|:--|:--|
| `compute_cte()` | (pos, path) → float | 経路からの横方向誤差を算出 | path_follow.py |
| `compute_theta_err()` | (heading, tangent) → float | 車両姿勢と経路接線の角度差を返す | path_follow.py |
| `pose_control()` | (state, goal) → (v, δ) | 目標地点へ収束するための制御入力を生成 | pose_regulation.py |
| `optimize_params()` | (particles, loss_fn) → best_params | PSOによる制御パラメータ最適化 | pso_adapt.py |

---

## 🧠 AI補助向けメモ
- この階層のモジュールは、**“数理的ロジック”と“調整ロジック”を分離**して設計されている。  
  - 数理的部分（誤差計算、角度操作）は `models.geometry` に委譲。  
  - チューニング部分（PSO・強化学習）は独立モジュールとして組込可能。  
- AIに依頼する場合の指針：  
  - **「制御則の改良やパラメータ最適化」** → この階層を対象に議論  
  - **「物理挙動や運動更新」** → `models` を参照  
  - **「経路生成や座標系」** → `trajectory` を参照

---