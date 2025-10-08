# ⚙️ RealVehicle 用パラメータ一覧

このドキュメントは、Raspberry Pi + I2C モータ制御基板と通信する際に利用される
`param_ids.py` の主要パラメータを、機能別に整理したものです。  
本一覧は `models/real_vehicle.py` におけるパラメータ対応の基準として利用します。

---

## 🧩 パラメータ分類表

| 分類 | パラメータ名 | ID | 型 | 範囲 / 単位 | 機能・説明 |
|:--|:--|:--:|:--|:--|:--|
| **基本制御** | `VEL_LEFT` | 0x01 | int16 | -255〜255 | 左モータの速度指令（PWM値）。正負で回転方向を示す。 |
|  | `VEL_RIGHT` | 0x02 | int16 | -255〜255 | 右モータの速度指令（PWM値）。経路追従で速度制御に利用。 |
|  | `TARGET_SPEED` | 0x10 | int16 | 0〜255 | 並進速度の基準（m/s換算も可）。 |
|  | `TARGET_STEER` | 0x11 | int16 | -90〜90 | 目標ステア角（度）。RealVehicle の `delta` に対応。 |
| **エンコーダ / 状態監視** | `ENC_L` | 0x20 | uint16 | カウント | 左エンコーダの累積カウント値。 |
|  | `ENC_R` | 0x21 | uint16 | カウント | 右エンコーダの累積カウント値。 |
|  | `VEL_MEAS_L` | 0x22 | int16 | -255〜255 | 左モータの現在速度（PWMスケール）。 |
|  | `VEL_MEAS_R` | 0x23 | int16 | -255〜255 | 右モータの現在速度（PWMスケール）。 |
|  | `BATTERY_VOLT` | 0x24 | uint16 | mV | バッテリ電圧モニタ。安全制御用。 |
|  | `TEMP_MOTOR` | 0x25 | uint16 | ℃ | モータ温度。過熱保護に利用可。 |
| **ゲイン / PID設定** | `KP_VEL` | 0x30 | float | 0〜2.0 | 速度制御用 P ゲイン。 |
|  | `KI_VEL` | 0x31 | float | 0〜1.0 | 速度制御用 I ゲイン。 |
|  | `KD_VEL` | 0x32 | float | 0〜1.0 | 速度制御用 D ゲイン。 |
|  | `KP_POS` | 0x33 | float | 0〜5.0 | 位置制御用 P ゲイン。 |
|  | `KI_POS` | 0x34 | float | 0〜1.0 | 位置制御用 I ゲイン。 |
|  | `KD_POS` | 0x35 | float | 0〜1.0 | 位置制御用 D ゲイン。 |
| **動作設定 / モード** | `ENABLE_MOTOR` | 0x40 | bool | 0/1 | モータ出力 ON/OFF。 |
|  | `CONTROL_MODE` | 0x41 | uint8 | 0〜3 | 0:直制御, 1:速度制御, 2:位置制御など。 |
|  | `SAFE_SHUTDOWN` | 0x42 | bool | 0/1 | 緊急停止。 |
| **パラメータ管理** | `SAVE_PARAMS` | 0x50 | bool | 0/1 | 現在設定を EEPROM に保存。 |
|  | `LOAD_PARAMS` | 0x51 | bool | 0/1 | EEPROM から設定を読み込み。 |
| **デバッグ / その他** | `FW_VERSION` | 0x60 | uint16 | - | ファームウェアバージョン。 |
|  | `DEVICE_STATUS` | 0x61 | uint16 | - | 通信状態やエラーコード。 |

---

## 🧭 RealVehicle クラス対応マッピング

| RealVehicle 内部関数 | 対応パラメータ | 機能・目的 |
|:--|:--|:--|
| `update_motor_output()` | `VEL_LEFT`, `VEL_RIGHT` | 左右モータへ PWM 速度を送信。差動二輪モデルの出力段。 |
| `_convert_velocity_to_pwm()` | `TARGET_SPEED` | m/s → PWM スケール変換の基準。 |
| `get_state()` | `ENC_L`, `ENC_R`, `VEL_MEAS_L`, `VEL_MEAS_R` | 実機の速度・位置情報を取得。 |
| `stop()` | `VEL_LEFT`, `VEL_RIGHT` (0送信) | 安全停止処理。 |
| `connect()` / `disconnect()` | `DEVICE_STATUS` | 通信状態確認・I2C 終了処理。 |
| `save_params()` / `load_params()` | `SAVE_PARAMS`, `LOAD_PARAMS` | チューニング内容を EEPROM に保存／復元。 |

---

## 💡 備考・運用指針

- **速度／角度制御**  
  差動二輪モデルに基づいて左右モータの PWM 値に変換し、`update_motor_output()` で送信します。

- **エンコーダ情報**  
  `ENC_L` / `ENC_R` は走行距離や姿勢推定（オドメトリ）にも利用可能です。

- **安全制御**  
  実機動作時には、`ENABLE_MOTOR` と `SAFE_SHUTDOWN` の状態を監視しておくことを推奨します。

- **チューニング**  
  PID ゲイン関連パラメータ（`KP_VEL`〜`KD_POS`）は GUI (`rpi_i2c_controller_gui_Tuning_v6.py`) によりリアルタイム調整可能です。

- **拡張**  
  実機側のファームウェアでパラメータIDが追加された場合は、このドキュメントと `param_ids.py` の両方を更新してください。

---

（更新日: YYYY-MM-DD / 対応ファームウェア: v6系）