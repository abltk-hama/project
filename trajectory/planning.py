# trajectory/planning_core.py
from __future__ import annotations
from dataclasses import dataclass
from typing import List, Tuple, Dict, Any, Optional
import math

Pose = Tuple[float, float, float]  # (x, y, yaw[rad])

@dataclass
class PlanConfig:
    step_m: float = 0.10         # ポリライン離散間隔 [m]
    pos_tol: float = 0.03        # 到達判定(位置)
    yaw_tol_deg: float = 5.0     # 到達判定(姿勢)
    hold_steps: int = 8          # 連続保持

def _wrap(a: float) -> float:
    while a > math.pi:  a -= 2*math.pi
    while a < -math.pi: a += 2*math.pi
    return a

def _line_points(start_xyz: List[float], end_xyz: List[float], step_m: float) -> List[Pose]:
    sx, sy, _ = start_xyz; ex, ey, _ = end_xyz
    L = math.hypot(ex - sx, ey - sy)
    n = max(1, int(max(1.0, L) / max(1e-6, step_m)))
    yaw = math.atan2(ey - sy, ex - sx)
    return [(sx + (i/n)*(ex - sx), sy + (i/n)*(ey - sy), yaw) for i in range(n+1)]

def _arc_points(center: List[float], radius: float, sa_deg: float, ea_deg: float,
                direction: str, step_m: float) -> List[Pose]:
    cx, cy = center
    sa = math.radians(sa_deg); ea = math.radians(ea_deg)
    ccw = (direction.upper() != "CW")
    def ang_diff(a0, a1, ccw=True):
        d = (a1 - a0) % (2*math.pi)
        if not ccw and d != 0.0: d -= 2*math.pi
        return d
    dth = ang_diff(sa, ea, ccw=ccw)
    arc_len = abs(dth) * radius
    n = max(1, int(max(1.0, arc_len) / max(1e-6, step_m)))
    pts: List[Pose] = []
    for i in range(n+1):
        t = i/n
        ang = sa + t*dth
        x = cx + radius*math.cos(ang); y = cy + radius*math.sin(ang)
        yaw = ang + (math.pi/2 if ccw else -math.pi/2)
        pts.append((x, y, _wrap(yaw)))
    return pts

class MissionPlan:
    """
    JSON elements をポリラインに変換し、セクション→点列の順で目標Poseを供給。
    本クラスは「経路管理」のみを担当し、制御は呼び出し側に委譲。
    """
    def __init__(self, route_data: Dict[str, Any], cfg: PlanConfig = PlanConfig()):
        self.cfg = cfg
        self.sections: List[List[Pose]] = self._build_sections(route_data.get("elements", []))
        self.sec_idx: int = 0
        self.pt_idx: int = 0
        self._hold: int = 0

    def _build_sections(self, elements: List[Dict[str, Any]]) -> List[List[Pose]]:
        secs: List[List[Pose]] = []
        for e in elements:
            et = e["type"].lower()
            if et == "line":
                secs.append(_line_points(e["start"], e["end"], self.cfg.step_m))
            elif et == "arc":
                secs.append(_arc_points(e["center"], float(e["radius"]),
                                        float(e["start_angle"]), float(e["end_angle"]),
                                        e.get("direction", "CCW"), self.cfg.step_m))
        return [s for s in secs if len(s) >= 1]

    def is_finished(self) -> bool:
        return self.sec_idx >= len(self.sections)

    def current_target(self) -> Optional[Pose]:
        if self.is_finished(): return None
        return self.sections[self.sec_idx][self.pt_idx]

    def update_reached(self, state: Pose) -> bool:
        """
        呼び出し側で制御・状態更新後に呼ぶ。
        到達(位置+姿勢)を連続保持できたら True を返し、次の目標へ進める。
        """
        tgt = self.current_target()
        if tgt is None: return True
        pos_ok = math.hypot(state[0]-tgt[0], state[1]-tgt[1]) < self.cfg.pos_tol
        yaw_ok = abs(_wrap(tgt[2]-state[2])) < math.radians(self.cfg.yaw_tol_deg)
        if pos_ok and yaw_ok:
            self._hold += 1
            if self._hold >= self.cfg.hold_steps:
                self._advance()
                self._hold = 0
                return True
        else:
            self._hold = 0
        return False

    def _advance(self) -> None:
        """次の点へ。セクション終端なら次セクションの始点へ。"""
        self.pt_idx += 1
        if self.pt_idx >= len(self.sections[self.sec_idx]):
            self.sec_idx += 1
            self.pt_idx = 0
