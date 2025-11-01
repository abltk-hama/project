from trajectory.trajectory_loader import TrajectoryLoader
from trajectory.planning import MissionPlan, PlanConfig
from models.debug import FirstOrderTracker
import matplotlib.pyplot as plt

p=__import__('pathlib').Path('.')/"plan_test.json"
route = {
    "trajectory_id":"square_demo",
    "elements":[
        {"type":"line","start":[0,0,0],"end":[1,0,0],"speed":0.2},
        {"type":"line","start":[1,0,0],"end":[1,1,90],"speed":0.2},
        {"type":"line","start":[1,1,90],"end":[0,1,180],"speed":0.2},
        {"type":"line","start":[0,1,180],"end":[0,0,-90],"speed":0.2},
    ]
}

#route = TrajectoryLoader.load_json(path=str(p), rotation_deg=0, scale=1.0, translate=(0.0, 0.0))
plan = MissionPlan(route, PlanConfig(step_m=0.10))
tracker = FirstOrderTracker(dt=0.02)

state_history = []  # 軌跡を記録
state = (0.0, 0.0, 0.0)
while not plan.is_finished():
    tgt = plan.current_target()
    state = tracker.step(state, tgt)  # 仮の状態更新
    plan.update_reached(state)
    state_history.append([state[0], state[1]])

state_x, state_y = zip(*state_history)

plt.figure(figsize=(8, 6))
plt.plot(state_x, state_y, 'r-', label="Actual Path")  # 追従した軌道
plt.legend()
plt.xlabel("X [m]")
plt.ylabel("Y [m]")
plt.title("Trajectory Tracking Simulation")
plt.grid(True)
plt.show()
