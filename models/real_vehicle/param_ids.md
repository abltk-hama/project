# param_ids.py 定義パラメータ一覧

本一覧は **`param_ids.py` の `PARAMS` / `GROUPS` 定義のみ**を反映しています。グループ順は `GROUPS` に準拠。

> 列の意味: **ID**（hex）, **型**, **最小**, **最大**, **説明（短）**。必要に応じて各グループ末尾に**詳細説明**も追記しています。

---

## PID — PIDゲイン調整

| ID | 名称 | 型 | 最小 | 最大 | 説明（短） |
|---|---|---|---|---|---|
| 0x01 | KP | float | 0.0 | 500000.0 | 比例ゲイン：誤差に比例した補正。 |
| 0x02 | KI | float | 0.0 | 500000.0 | 積分ゲイン：定常誤差を解消。 |
| 0x03 | KD | float | 0.0 | 500000.0 | 微分ゲイン：振動抑制。 |

**詳細**
- **KP**: KP（比例ゲイン）は誤差に比例して制御出力を調整。大きいと応答は速いが振動しやすい。小さすぎると追従が遅い。調整は小さく始めて徐々に上げる。
- **KI**: 誤差を時間積分して補正。小さいと定常誤差が残り、大きいと遅れや発振。KP調整後に追加調整。
- **KD**: 誤差の変化率で制御。ノイズに敏感なので入れ過ぎ注意。KP/KI決定後に軽く付与。

---

## FF — フィードフォワード

| ID | 名称 | 型 | 最小 | 最大 | 説明（短） |
|---|---|---|---|---|---|
| 0x10 | FF_L_K0 | float | 0.0 | 65535.0 | 左FF定数項。 |
| 0x11 | FF_L_K1 | float | 0.0 | 999.0 | 左FF速度比例。 |
| 0x12 | FF_R_K0 | float | 0.0 | 65535.0 | 右FF定数項。 |
| 0x13 | FF_R_K1 | float | 0.0 | 999.0 | 右FF速度比例。 |
| 0x14 | FF_MAX_DUTY | int | 0 | 65535 | FF Duty上限。 |

**詳細**
- **FF_L/R_K0**: 各輪の無負荷損失を補償する定数項。
- **FF_L/R_K1**: 目標速度に比例して加算する係数。過大は不安定化に注意。
- **FF_MAX_DUTY**: フィードフォワード加算Dutyの上限。

---

## Assist — アシスト制御

| ID | 名称 | 型 | 最小 | 最大 | 説明（短） |
|---|---|---|---|---|---|
| 0x20 | ASSIST_ERR_ON_HZ | float | 0.0 | 500.0 | アシスト開始閾値[Hz]。 |
| 0x21 | ASSIST_ERR_OFF_HZ | float | 0.0 | 500.0 | アシスト終了閾値[Hz]。 |
| 0x22 | ASSIST_DUTY_BASE | int | 0 | 65535 | アシスト基本Duty。 |
| 0x23 | ASSIST_DUTY_K | float | 0.0 | 1000.0 | アシスト比例係数。 |
| 0x24 | ASSIST_DUTY_CAP | int | 0 | 65535 | アシストDuty上限。 |

**詳細**
- **ERR_ON/OFF_HZ**: 誤差が閾値をまたぐとアシスト開始/終了。
- **DUTY_BASE**: アシスト時に常に加算する固定Duty。
- **DUTY_K**: 誤差Hzに比例して追加するDuty係数。
- **DUTY_CAP**: アシストで追加できる最大Duty。

---

## Ratio — 左右比制御

| ID | 名称 | 型 | 最小 | 最大 | 説明（短） |
|---|---|---|---|---|---|
| 0x30 | RATIO_KP | float | 0.0 | 5.0 | 左右比制御ゲイン。 |
| 0x31 | RATIO_MAX_BIAS | float | 0.0 | 1.0 | 最大補正Duty比率。 |
| 0x32 | RATIO_REDUCE_FRAC | float | 0.0 | 1.0 | 反対側減衰率。 |
| 0x33 | RATIO_DEADBAND | float | 0.0 | 0.5 | デッドバンド。 |
| 0x34 | RATIO_MIN_TARGET_HZ | float | 0.0 | 200.0 | 有効最小ターゲットHz。 |

**詳細**
- **RATIO_KP**: 速度比の偏りを補正するゲイン。
- **MAX_BIAS**: 片側に与える補正の上限（比率）。
- **REDUCE_FRAC**: 片側を上げたとき相手側を下げる割合。
- **DEADBAND**: 比誤差がこの範囲内なら補正しない。
- **MIN_TARGET_HZ**: 比制御を有効化する最小ターゲットHz。

---

## Angle — 角度制御

| ID | 名称 | 型 | 最小 | 最大 | 説明（短） |
|---|---|---|---|---|---|
| 0x40 | ANGLE_OFFSET_MDEG | int | -180000 | 180000 | 角度オフセット[mdeg]。 |
| 0x41 | ANGLE_TAN_LIMIT_DEG | float | 10.0 | 85.0 | tan特異点クリップ角。 |
| 0x42 | ANGLE_MODE | enum | — | — | 角度モード（classic/theta_dir/dir_cos）。 |

**詳細**
- **OFFSET**: 取り付け誤差等の補正用オフセット。
- **TAN_LIMIT_DEG**: tan計算の発散を防ぐ角度上限。
- **MODE**: 角度指令の解釈方式（classic / θ+符号 / cos符号）。

---

## Speed — 速度上限/停止

| ID | 名称 | 型 | 最小 | 最大 | 説明（短） |
|---|---|---|---|---|---|
| 0x50 | SPEED_CAP_KPH_MAX | float | 0.0 | 32767.0 | 速度上限[km/h]。 |
| 0x51 | STOP_TARGET_HZ | float | 0.0 | 10.0 | 停止ゲート閾値[Hz]。 |

**詳細**
- **SPEED_CAP_KPH_MAX**: 安全のための速度上限制御。
- **STOP_TARGET_HZ**: このHz未満でStopGateを発動（Duty=0）。

---

## Speed Measurement — 速度計測調整

| ID | 名称 | 型 | 最小 | 最大 | 説明（短） |
|---|---|---|---|---|---|
| 0x60 | ENC_MAX_VALID_HZ | float | 0.0 | 1000.0 | 有効Hz上限（スパイク除外）。 |
| 0x61 | SPEED_TIMEOUT_US | int | 0 | 5000000 | 速度タイムアウト[µs]。 |
| 0x62 | ZERO_HOLD_MS | int | 0 | 5000 | ゼロ保持時間[ms]。 |
| 0x63 | PULSE_AVG_COUNT | int | 1 | 64 | 移動平均個数。 |
| 0x64 | SOFT_ALPHA_UNLOCKED | float | 0.0 | 1.0 | 解錠時フィルタ係数。 |
| 0x65 | ENC_SOFT_JUMP_HZ | float | 0.0 | 100.0 | ジャンプ抑制幅[Hz]。 |
| 0x66 | GRACE_JUMP_GAIN | float | 0.1 | 5.0 | 大段階緩和倍率。 |
| 0x67 | LOCK_MIN_HZ_VALID | float | 0.0 | 200.0 | LOCK成立最小Hz。 |
| 0x68 | LOCK_STREAK_N | int | 1 | 10 | LOCK連続回数。 |
| 0x69 | SPEED_TIMEOUT_LOCKED_MS | int | 0 | 5000 | LOCK時タイムアウト[ms]。 |
| 0x6A | SPEED_TIMEOUT_UNLOCKED_MS | int | 0 | 5000 | UNLOCK時タイムアウト[ms]。 |

**詳細**
- **ENC_MAX_VALID_HZ**: 上限超え測定をスパイクとして無視。
- **SPEED_TIMEOUT_US**: 無パルス継続後に速度=0。
- **ZERO_HOLD_MS**: Duty=0時に速度0を保持する時間。
- **PULSE_AVG_COUNT**: 速度平滑化のサンプル数。
- **SOFT_ALPHA_UNLOCKED**: UNLOCK時のα平滑係数。
- **ENC_SOFT_JUMP_HZ**: LOCK時に許容するΔジャンプ幅。
- **GRACE_JUMP_GAIN**: 目標変更直後の一時的緩和倍率。
- **LOCK_MIN_HZ_VALID / LOCK_STREAK_N**: LOCK成立条件（最小Hzと連続回数）。
- **SPEED_TIMEOUT_LOCKED/UNLOCKED_MS**: 状態別の速度受理タイムアウト。
