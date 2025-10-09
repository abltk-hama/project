# [MODULE]: models
[PURPOSE]:
  差動二輪ロボットの物理モデル・幾何計算・角度処理など、運動を支える基礎層。
  シミュレーションおよび実機制御の両方で共通して利用される。

[RESPONSIBILITY]:
  - 車両運動方程式による状態更新
  - 姿勢角・角速度の正規化
  - 距離・方位などの幾何的関係計算

[DEPENDS_ON]:
  - utils.math_tools  （数値計算の補助）
  - config.constants   （車両パラメータ）

[USED_BY]:
  - controllers.*     （制御層）
  - sim.vehicle_manager

---

## 📂 ファイル一覧

| ファイル名 | 概要 | 主な関数・クラス |
|:--|:--|:--|
| `physics.py` | 差動二輪モデルの運動計算。ステアリング角と速度から次状態を求める。 | `update_state()`, `compute_wheel_speeds()`, `simulate_dynamics()` |
| `geometry.py` | 幾何的関係（角度、距離、誤差）の計算を担当。 | `normalize_angle()`, `compute_distance()`, `angle_between()` |
| `real_vehicle.py` | 実機車両とのインターフェース。I2C通信を利用してモーターを制御。 | `RealVehicle`, `set_motor_speed()`, `update_from_sensor()` |

---

## ⚙️ インターフェース仕様

| 入力 | 出力 | 補足 |
|:--|:--|:--|
| `(v, δ)` | 並進速度・ステアリング角 | controllers から受け取る |
| `(vL, vR)` | 左右ホイール速度 | 物理モデル内部で算出 |
| `(x, y, θ)` | 新しい車両状態 | sim へ返却 |

---

## 🧩 関数リスト（全体要約）

| 関数 / クラス名 | 入出力 | 概要 | 所属ファイル |
|:--|:--|:--|:--|
| `update_state()` | (x, y, θ, v, δ, dt) → (x’, y’, θ’) | 次時刻の位置と角度を更新 | physics.py |
| `normalize_angle()` | (angle) → angle | [-π, π] に正規化 | geometry.py |
| `compute_distance()` | (p1, p2) → float | 2点間の距離を返す | geometry.py |
| `RealVehicle` | class | 実機制御用インターフェース | real_vehicle.py |

---

## 🧠 AI補助向けメモ
- `physics.py` と `geometry.py` は副作用のない純粋計算関数群。  
- `real_vehicle.py` のみ外部通信（I2C）を伴う。  
- AIに依頼する場合：
  - **運動方程式の拡張** → `physics` を対象  
  - **誤差や角度の算出ロジック** → `geometry` を対象  
  - **実機通信** → `real_vehicle` のみに限定（安全のため）  