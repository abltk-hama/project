# runner/runner_api.py（実装骨子）
from trajectory.trajectory_loader import load_json
from trajectory.planning import MissionPlan, PlanConfig
from models.debug import FirstOrderTracker
# from models.physics import PhysicsModel
# from models.real_vehicle import RealVehicle
import math
from dataclasses import dataclass
from typing import Optional, Tuple

@dataclass
class RouteTransform:
    rotation_deg: float = 0.0
    scale: float = 1.0
    translate: Tuple[float, float] = (0.0, 0.0)
    step_m: float = 0.10               # ポリライン間隔

@dataclass
class Termination:
    pos_tol: float = 0.03
    yaw_tol_deg: float = 5.0
    hold_steps: int = 8

@dataclass
class DebugModelParam:
    dt: float = 0.02
    tau_pos: float = 0.30
    tau_yaw: float = 0.25

@dataclass
class RunnerConfig:
    route_path: str
    transform: RouteTransform = RouteTransform()
    initial_pose: Tuple[float,float,float] = (0.0,0.0,0.0)
    model: str = "debug"               # "debug" | "physics" | "real"
    debug_param: DebugModelParam = DebugModelParam()
    termination: Termination = Termination()
    render_xy: bool = True
    render_dist: bool = True
    render_theta: bool = True
    save_csv: Optional[str] = None     # 例: "runs/run_YYYYMMDD_HHMMSS.csv"

def run_mission(cfg: RunnerConfig):
    """
    1) 経路読み込み＋幾何変換 → MissionPlan生成
    2) モデル選択（debug/physics/real）
    3) ループ（target取得→モデルで状態更新/観測→到達判定）
    4) ログ返却（GUI側で描画/保存）
    """
    # ここに実装（下の骨子参照）

def run_mission(cfg: RunnerConfig):
    route = load_json(cfg.route_path,
                      rotation_deg=cfg.transform.rotation_deg,
                      scale=cfg.transform.scale,
                      translate=cfg.transform.translate)
    plan = MissionPlan(route, PlanConfig(step_m=cfg.transform.step_m,
                                         pos_tol=cfg.termination.pos_tol,
                                         yaw_tol_deg=cfg.termination.yaw_tol_deg,
                                         hold_steps=cfg.termination.hold_steps))
    # モデル切替
    if cfg.model == "debug":
        model = FirstOrderTracker(cfg.debug_param.dt, cfg.debug_param.tau_pos, cfg.debug_param.tau_yaw)
        state = cfg.initial_pose
        get_state = lambda s: s
        step_fn = lambda s,tgt: model.step(s,tgt)
    elif cfg.model == "physics":
        model = PhysicsModel(...)             # 将来
        state = model.initial_state()
        get_state = model.get_state
        step_fn = lambda s,tgt: model.step(tgt)
    else:  # "real"
        hw = RealVehicle(...)                 # 将来（非常停止等の安全対策は別途）
        state = hw.read_pose()
        get_state = lambda _: hw.read_pose()
        step_fn = lambda s,tgt: hw.command_to(tgt)

    # ループ
    logs = []
    while not plan.is_finished():
        tgt = plan.current_target()           # (x*,y*,yaw*)
        state = step_fn(state, tgt)
        pose = get_state(state)
        plan.update_reached(pose)
        logs.append((pose, tgt))

    # 戻り値（GUI側で描画/保存）
    return {"completed": True, "logs": logs}
