import numpy as np
#from __future__ import annotations
import json, math
from typing import Dict, Any, List, Tuple

class TrajectoryLoader:
    def __init__(self, t_end=20, num_points=100):
        """ 軌道データを動的に生成 """
        self.t_end = t_end
        self.num_points = num_points
        self.t = np.linspace(0, t_end, num_points)

    def sine_wave(self, amplitude=5, frequency=0.2):
        """ サインカーブ軌道を生成 """
        x = self.t
        y = amplitude * np.sin(frequency * self.t)
        return np.vstack([x, y]).T  # (N, 2) の形に変換
    
    def arc(self, scale=5, start_radians=0.0, range_radians=np.pi/2):
        """ 円弧軌道を生成 """
        t = range_radians * self.t / self.t_end
        x = scale * (1 - np.cos(t+start_radians))
        y = scale * np.sin(t+start_radians)
        return np.vstack([x, y]).T  # (N, 2) の形に変換

    def figure_eight(self, scale=5):
        """ 8の字軌道を生成 """
        x = scale * np.sin(self.t)
        y = scale * np.sin(self.t) * np.cos(self.t)
        return np.vstack([x, y]).T  # (N, 2) の形に変換

    def spiral(self, scale=1):
        """ らせん軌道を生成 """
        x = scale * self.t * np.cos(self.t)
        y = scale * self.t * np.sin(self.t)
        return np.vstack([x, y]).T  # (N, 2) の形に変換
    
    def load_json(self, path: str,
                rotation_deg: float = 0.0,
                scale: float = 1.0,
                translate: Tuple[float, float] = (0.0, 0.0),
                scale_speed: bool = False) -> Dict[str, Any]:
        """JSONを読み込み、スキーマ検証後に幾何変換を適用して返す:contentReference[oaicite:12]{index=12}"""
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        validate_schema(data)
        transform_inplace(data, rotation_deg, scale, translate, scale_speed)
        return data
    
# ========== 1) schema validate ==========
def _assert(cond: bool, msg: str) -> None:
    if not cond:
        raise ValueError(f"[trajectory_loader] {msg}")

def validate_schema(data: Dict[str, Any]) -> None:
    _assert(isinstance(data, dict), "root must be dict")
    _assert("trajectory_id" in data and isinstance(data["trajectory_id"], str), "trajectory_id(str) missing")
    _assert("elements" in data and isinstance(data["elements"], list), "elements(list) missing")

    for i, e in enumerate(data["elements"]):
        _assert("type" in e and isinstance(e["type"], str), f"elements[{i}].type missing")
        et = e["type"].lower()
        _assert(et in {"line", "arc"}, f"elements[{i}].type invalid: {e['type']}")

        if et == "line":
            _assert("start" in e and isinstance(e["start"], list) and len(e["start"]) >= 2, f"elements[{i}].start invalid")
            _assert("end" in e and isinstance(e["end"], list) and len(e["end"]) >= 2, f"elements[{i}].end invalid")
        else:  # arc
            _assert("center" in e and isinstance(e["center"], list) and len(e["center"]) == 2, f"elements[{i}].center invalid")
            _assert("radius" in e and isinstance(e["radius"], (int, float)) and e["radius"] > 0, f"elements[{i}].radius invalid")
            _assert("start_angle" in e and "end_angle" in e, f"elements[{i}].angles missing")

# ========== 2) geometry transform ==========
def _rot2d(x: float, y: float, rad: float) -> Tuple[float, float]:
    c, s = math.cos(rad), math.sin(rad)
    return c*x - s*y, s*x + c*y

def _norm_angle_deg(a: float) -> float:
    a = a % 360.0
    return a if a >= 0.0 else a + 360.0

def _apply_point(p: List[float], rad: float, s: float, tx: float, ty: float) -> List[float]:
    """[x,y,(theta_deg?)] を変換。theta_degがあれば回転角を加算。"""
    x, y = p[0]*s, p[1]*s
    x, y = _rot2d(x, y, rad)
    out = [x + tx, y + ty]
    if len(p) >= 3:
        out.append(_norm_angle_deg(p[2]))  # とりあえず正規化のみ（回転は呼び出し側で加算）
    return out

def transform_inplace(data: Dict[str, Any],
                    rotation_deg: float = 0.0,
                    scale: float = 1.0,
                    translate: Tuple[float, float] = (0.0, 0.0),
                    scale_speed: bool = False) -> None:
    """JSONスキーマ準拠の構造へ幾何変換を適用（破壊的）:contentReference[oaicite:11]{index=11}"""
    rad = math.radians(rotation_deg)
    tx, ty = translate
    for e in data.get("elements", []):
        et = e.get("type", "").lower()
        if et == "line":
            if "start" in e:
                e["start"] = _apply_point(e["start"], rad, scale, tx, ty)
                if len(e["start"]) >= 3:
                    e["start"][2] = _norm_angle_deg(e["start"][2] + rotation_deg)
            if "end" in e:
                e["end"] = _apply_point(e["end"], rad, scale, tx, ty)
                if len(e["end"]) >= 3:
                    e["end"][2] = _norm_angle_deg(e["end"][2] + rotation_deg)
        elif et == "arc":
            if "center" in e:
                cx, cy = e["center"]
                cx, cy = _rot2d(cx*scale, cy*scale, rad)
                e["center"] = [cx + tx, cy + ty]
            if "radius" in e:
                e["radius"] = float(e["radius"]) * scale
            if "start_angle" in e:
                e["start_angle"] = _norm_angle_deg(e["start_angle"] + rotation_deg)
            if "end_angle" in e:
                e["end_angle"] = _norm_angle_deg(e["end_angle"] + rotation_deg)
            if "direction" in e and isinstance(e["direction"], str):
                e["direction"] = e["direction"].upper()
        # 任意：速度のスケール
        if scale_speed and "speed" in e and isinstance(e["speed"], (int, float)):
            e["speed"] = float(e["speed"]) * scale
