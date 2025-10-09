# [MODULE]: models
[PURPOSE]:
  車両モデルや実機制御モデルを管理する階層。  
  物理モデル (`physics.py`) と実機モデル (`real_vehicle.py`) は、  
  共通のインターフェース `interfaces.py` を介して同一構文で扱える。

---

## [FILES & RESPONSIBILITIES]  

### physics.py  
- 差動二輪ロボットの運動モデルを記述。  
- 入力：ステア角 δ [rad]、速度 v [m/s]  
- 出力：位置 (x, y)、姿勢角 θ、角速度 ω。  
- `update_state()`・`get_state()`・`stop()`・`reset()` を共通で実装。

---

### real_vehicle/real_vehicle.py  
- 実機制御用のI2Cベースの車両クラス。  
- `RealVehicle` クラスは `VehicleInterface` を継承。  
- `update_state()` でモータ指令を送信。  
- `get_state()` でエンコーダ等から状態を取得。

---

### real_vehicle/param_defs.py  
- 実機の制御パラメータ定義ファイル。  
- 旧 `param_ids.py` を統合・再構成。  
- ハード依存のパラメータIDやデフォルト値を定義。

#### 主な関数
| 関数名 | 説明 |
|---------|------|
| `get_param_by_id(param_id)` | IDからパラメータ名を逆引き |
| `list_params(group)` | グループ内のパラメータ一覧取得 |
| `describe_param(name)` | パラメータ説明の生成 |

#### 主な定数
| 定数名 | 説明 |
|---------|------|
| `PARAMS` | パラメータID・デフォルト値・説明を保持する辞書 |
| `GROUPS` | パラメータ分類（PID, FF, Ratioなど）を保持 |

---

### interfaces.py  
- シミュレーションと実機制御の共通インターフェース。  
- `VehicleInterface` 抽象クラスとして定義。

#### 主なメソッド
| メソッド | 説明 |
|-----------|------|
| `update_state(delta, v)` | ステア角・速度で車両状態を更新 |
| `get_state()` | x, y, θ, v, ω 等を辞書で返す |
| `stop()` | モータ出力停止 |
| `reset()` | 状態リセット・通信再初期化 | 

---

## [STRUCTURE SUMMARY]  
```
models/
├── interfaces.py
├── physics.py
└── real_vehicle/
    ├── real_vehicle.py
    └── param_defs.py
```

---

## [UPDATE LOG]
- **2025-10-09**  
  - `interfaces.py` 新規追加。  
  - `real_vehicle/param_defs.py` に `param_ids.py` の定義を統合。  
  - `physics.py` / `real_vehicle.py` が共通API化。