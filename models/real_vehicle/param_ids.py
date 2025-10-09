# -*- coding: utf-8 -*-
"""
param_ids.py — I2C調整対象パラメータと説明文定義
GUI側はこれをインポートして表示・説明に利用する
"""

PARAMS = {
    # ==== PID ====
    0x01: {
        "name": "KP", "type": "float", "min": 0.0, "max": 500000.0,
        "desc_short": "比例ゲイン：誤差に比例した補正。",
        "desc_long": (
            "KP（比例ゲイン）は誤差に比例して制御出力を調整します。\n"
            "大きくすると応答は速くなりますが、振動やオーバーシュートが増えやすくなります。\n"
            "小さすぎると応答が遅れ、目標に追従しづらくなります。\n"
            "調整方法: 小さい値から徐々に上げ、応答性と安定性のバランスを探してください。"
        )
    },
    0x02: {
        "name": "KI", "type": "float", "min": 0.0, "max": 500000.0,
        "desc_short": "積分ゲイン：定常誤差を解消。",
        "desc_long": (
            "KI（積分ゲイン）は誤差を時間積分して補正します。\n"
            "小さすぎると定常誤差が残り、大きすぎると遅れや振動が発生します。\n"
            "調整方法: KPで大まかに合わせた後、KIを上げて誤差を解消します。"
        )
    },
    0x03: {
        "name": "KD", "type": "float", "min": 0.0, "max": 500000.0,
        "desc_short": "微分ゲイン：振動抑制。",
        "desc_long": (
            "KD（微分ゲイン）は誤差の変化率を用いて制御します。\n"
            "ノイズに敏感なので過大にすると不安定化します。\n"
            "調整方法: KP, KIを決めた後、振動を抑える程度に設定してください。"
        )
    },

    # ==== FF ====
    0x10: {
        "name": "FF_L_K0", "type": "float", "min": 0.0, "max": 65535.0,
        "desc_short": "左FF定数項。",
        "desc_long": "左モータの無負荷損失を補償するための定数項です。"
    },
    0x11: {
        "name": "FF_L_K1", "type": "float", "min": 0.0, "max": 999.0,
        "desc_short": "左FF速度比例。",
        "desc_long": "左モータの速度に比例してDutyを補う係数です。"
    },
    0x12: {
        "name": "FF_R_K0", "type": "float", "min": 0.0, "max": 65535.0,
        "desc_short": "右FF定数項。",
        "desc_long": "右モータの無負荷損失を補償するための定数項です。"
    },
    0x13: {
        "name": "FF_R_K1", "type": "float", "min": 0.0, "max": 999.0,
        "desc_short": "右FF速度比例。",
        "desc_long": "右モータの速度に比例してDutyを補う係数です。"
    },
    0x14: {
        "name": "FF_MAX_DUTY", "type": "int", "min": 0, "max": 65535,
        "desc_short": "FF Duty上限。",
        "desc_long": "フィードフォワードで加算されるDutyの上限値です。"
    },

    # ==== Assist ====
    0x20: {
        "name": "ASSIST_ERR_ON_HZ", "type": "float", "min": 0.0, "max": 500.0,
        "desc_short": "アシスト開始閾値[Hz]。",
        "desc_long": "誤差がこのHzを超えるとアシストDutyを追加します。"
    },
    0x21: {
        "name": "ASSIST_ERR_OFF_HZ", "type": "float", "min": 0.0, "max": 500.0,
        "desc_short": "アシスト終了閾値[Hz]。",
        "desc_long": "誤差がこのHz未満になるとアシストを停止します。"
    },
    0x22: {
        "name": "ASSIST_DUTY_BASE", "type": "int", "min": 0, "max": 65535,
        "desc_short": "アシスト基本Duty。",
        "desc_long": "アシスト時に常に加算する基本Duty値です。"
    },
    0x23: {
        "name": "ASSIST_DUTY_K", "type": "float", "min": 0.0, "max": 1000.0,
        "desc_short": "アシスト比例係数。",
        "desc_long": "誤差Hzに比例して追加するDutyの係数です。"
    },
    0x24: {
        "name": "ASSIST_DUTY_CAP", "type": "int", "min": 0, "max": 65535,
        "desc_short": "アシストDuty上限。",
        "desc_long": "アシストで追加されるDutyの最大値です。"
    },

    # ==== Ratio ====
    0x30: {
        "name": "RATIO_KP", "type": "float", "min": 0.0, "max": 5.0,
        "desc_short": "左右比制御ゲイン。",
        "desc_long": "左右の速度比を補正するためのゲインです。"
    },
    0x31: {
        "name": "RATIO_MAX_BIAS", "type": "float", "min": 0.0, "max": 1.0,
        "desc_short": "最大補正Duty比率。",
        "desc_long": "左右比制御で加える補正量の上限（比率）です。"
    },
    0x32: {
        "name": "RATIO_REDUCE_FRAC", "type": "float", "min": 0.0, "max": 1.0,
        "desc_short": "反対側減衰率。",
        "desc_long": "一方を補正したとき、もう一方をどの割合で減衰させるか。"
    },
    0x33: {
        "name": "RATIO_DEADBAND", "type": "float", "min": 0.0, "max": 0.5,
        "desc_short": "デッドバンド。",
        "desc_long": "比誤差がこの範囲内なら補正を行わない。"
    },
    0x34: {
        "name": "RATIO_MIN_TARGET_HZ", "type": "float", "min": 0.0, "max": 200.0,
        "desc_short": "有効最小ターゲットHz。",
        "desc_long": "比制御を有効にする最小ターゲットHzです。"
    },

    # ==== Angle ====
    0x40: {
        "name": "ANGLE_OFFSET_MDEG", "type": "int", "min": -180000, "max": 180000,
        "desc_short": "角度オフセット[mdeg]。",
        "desc_long": "車体構造や取り付け誤差を補正するための角度オフセットです。"
    },
    0x41: {
        "name": "ANGLE_TAN_LIMIT_DEG", "type": "float", "min": 10.0, "max": 85.0,
        "desc_short": "tan特異点クリップ角[deg]。",
        "desc_long": "tan計算の発散を防ぐための角度上限値です。"
    },
    0x42: {
        "name": "ANGLE_MODE", "type": "enum", "enum": ["classic","theta_dir","dir_cos"],
        "desc_short": "角度モード。",
        "desc_long": (
            "角度指令の解釈モードです。\n"
            "classic: 従来方式\n"
            "theta_dir: θで方向、符号で進退を決める\n"
            "dir_cos: cosθの符号で進退を決める"
        )
    },

    # ==== Speed ====
    0x50: {
        "name": "SPEED_CAP_KPH_MAX", "type": "float", "min": 0.0, "max": 32767.0,
        "desc_short": "速度上限[km/h]。",
        "desc_long": "走行速度の上限です。安全のため必ず設定してください。"
    },
    0x51: {
        "name": "STOP_TARGET_HZ", "type": "float", "min": 0.0, "max": 10.0,
        "desc_short": "停止ゲート閾値[Hz]。",
        "desc_long": "このHz未満になったら停止ゲートを発動し、Dutyを0にします。"
    },

    # ==== Speed Measurement ====
    0x60: {
        "name": "ENC_MAX_VALID_HZ", "type": "float", "min": 0.0, "max": 1000.0,
        "desc_short": "有効Hz上限。",
        "desc_long": "この値を超える測定値はスパイクとして無視します。"
    },
    0x61: {
        "name": "SPEED_TIMEOUT_US", "type": "int", "min": 0, "max": 5000000,
        "desc_short": "速度タイムアウト[µs]。",
        "desc_long": "パルスがこの時間以上来なければ速度=0と判定します。"
    },
    0x62: {
        "name": "ZERO_HOLD_MS", "type": "int", "min": 0, "max": 5000,
        "desc_short": "ゼロ保持時間[ms]。",
        "desc_long": "Duty=0時に測定値をゼロに保持する時間です。"
    },
    0x63: {
        "name": "PULSE_AVG_COUNT", "type": "int", "min": 1, "max": 64,
        "desc_short": "移動平均個数。",
        "desc_long": "速度算出に使う移動平均のサンプル数です。"
    },
    0x64: {
        "name": "SOFT_ALPHA_UNLOCKED", "type": "float", "min": 0.0, "max": 1.0,
        "desc_short": "解錠時フィルタ係数。",
        "desc_long": "UNLOCK状態のときに使うソフトフィルタの係数αです。"
    },
    0x65: {
        "name": "ENC_SOFT_JUMP_HZ", "type": "float", "min": 0.0, "max": 100.0,
        "desc_short": "ジャンプ抑制幅[Hz]。",
        "desc_long": "LOCK状態で許容する測定値のジャンプ幅です。"
    },
    0x66: {
        "name": "GRACE_JUMP_GAIN", "type": "float", "min": 0.1, "max": 5.0,
        "desc_short": "大段階緩和倍率。",
        "desc_long": "目標変更時に一時的にジャンプ幅を緩和する倍率です。"
    },
    0x67: {
        "name": "LOCK_MIN_HZ_VALID", "type": "float", "min": 0.0, "max": 200.0,
        "desc_short": "LOCK成立最小Hz。",
        "desc_long": "LOCK状態に入るために必要な最低Hzです。"
    },
    0x68: {
        "name": "LOCK_STREAK_N", "type": "int", "min": 1, "max": 10,
        "desc_short": "LOCK連続回数。",
        "desc_long": "LOCK成立に必要な連続パルス回数です。"
    },
    0x69: {
        "name": "SPEED_TIMEOUT_LOCKED_MS", "type": "int", "min": 0, "max": 5000,
        "desc_short": "LOCK時タイムアウト[ms]。",
        "desc_long": "LOCK状態で速度=0と判定するまでの時間です。"
    },
    0x6A: {
        "name": "SPEED_TIMEOUT_UNLOCKED_MS", "type": "int", "min": 0, "max": 5000,
        "desc_short": "UNLOCK時タイムアウト[ms]。",
        "desc_long": "UNLOCK状態で速度=0と判定するまでの時間です。"
    },
}

# ==== グループ説明 ====
GROUPS = {
    "PID": {
        "desc_short": "PIDゲイン調整",
        "desc_long": (
            "比例・積分・微分の3要素で構成される基本制御です。\n"
            "KP: 応答速度、KI: 定常誤差解消、KD: 振動抑制。\n"
            "まずKPを調整し、次にKIで偏差を解消、最後にKDで安定化させます。"
        )
    },
    "FF": {
        "desc_short": "フィードフォワード",
        "desc_long": (
            "目標速度からDutyを直接算出して加える補助制御です。\n"
            "加速応答を改善しPIDの負担を軽減しますが、過大だと不安定化します。"
        )
    },
    "Assist": {
        "desc_short": "アシスト制御",
        "desc_long": (
            "誤差が一定以上のときにDutyを追加して立ち上がりを助けます。\n"
            "過大設定は暴走の原因となるので注意してください。"
        )
    },
    "Ratio": {
        "desc_short": "左右比制御",
        "desc_long": (
            "左右の速度比を一定に保つための補正制御です。\n"
            "片輪に負荷がかかったときの直進性を維持します。"
        )
    },
    "Angle": {
        "desc_short": "角度制御",
        "desc_long": (
            "角度θに基づいて左右輪の速度を決めます。\n"
            "θ=0°で前進、±180°で後退、±45°で旋回となります。"
        )
    },
    "Speed": {
        "desc_short": "速度上限/停止制御",
        "desc_long": (
            "走行速度の上限値と停止ゲート条件を管理するパラメータ群です。"
        )
    },
    "SpeedMeasure": {
        "desc_short": "速度計測調整",
        "desc_long": (
            "エンコーダ信号から速度を算出する際の調整パラメータ群です。\n"
            "スパイク除去・移動平均・LOCK判定条件などを調整できます。"
        )
    },
}
