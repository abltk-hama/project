"""
ハードウェア制御用パラメータ定義ファイル
-------------------------------------------------
本ファイルは、I2C経由で読み書き可能なパラメータ群を定義する。

【役割】
- 実機のPIDゲイン、フィードフォワード、補助係数、角度補正などを定義。
- 各パラメータは "ID（整数）" と "説明" をセットで持つ。
- 実機のI2C制御クラス（real_vehicle.py）から参照される。

※このファイルは構造情報のみを持ち、I2C通信ロジックは含まない。
"""

from typing import TypedDict

class ParamEntry(TypedDict):
    id: int
    name: str
    description: str
    default: float

# 各パラメータグループ
GROUPS = {
    "PID": ["KP", "KI", "KD"],
    "FF": ["FF_GAIN"],
    "ASSIST": ["ACC_GAIN", "DEC_GAIN"],
    "RATIO": ["GEAR_RATIO"],
    "ANGLE": ["OFFSET_ANGLE"],
}

# パラメータ定義本体
PARAMS: dict[str, ParamEntry] = {
    # PID関連
    "KP": {"id": 0x01, "name": "Proportional Gain", "description": "Pゲイン", "default": 1.0},
    "KI": {"id": 0x02, "name": "Integral Gain", "description": "Iゲイン", "default": 0.0},
    "KD": {"id": 0x03, "name": "Derivative Gain", "description": "Dゲイン", "default": 0.0},

    # フィードフォワード
    "FF_GAIN": {"id": 0x10, "name": "FeedForward Gain", "description": "速度FF補正", "default": 0.1},

    # アシスト・加減速制御
    "ACC_GAIN": {"id": 0x20, "name": "Acceleration Assist", "description": "加速補助係数", "default": 0.5},
    "DEC_GAIN": {"id": 0x21, "name": "Deceleration Assist", "description": "減速補助係数", "default": 0.5},

    # ギア比
    "GEAR_RATIO": {"id": 0x30, "name": "Gear Ratio", "description": "モータギア比", "default": 1.0},

    # 角度補正
    "OFFSET_ANGLE": {"id": 0x40, "name": "Angle Offset", "description": "センサー角度補正", "default": 0.0},
}

def get_param_by_id(param_id: int) -> str | None:
    """IDからパラメータ名を逆引きする"""
    for name, entry in PARAMS.items():
        if entry["id"] == param_id:
            return name
    return None

def list_params(group: str | None = None) -> list[str]:
    """グループ内のパラメータ一覧を返す"""
    if group and group in GROUPS:
        return GROUPS[group]
    return list(PARAMS.keys())

def describe_param(name: str) -> str:
    """パラメータの説明を返す"""
    entry = PARAMS.get(name)
    if entry:
        return f"[{name}] {entry['description']} (default={entry['default']})"
    return f"[{name}] Not defined."