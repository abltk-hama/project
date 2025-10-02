import numpy as np
import time
from config import Config
import simulation
from visualization import Visualization
from trajectory_loader import TrajectoryLoader
from optimization import Optimizer
from trajectory_control import TrajectoryController
from simple_predict import PredictCompute
from dubin_control import DubinsPath

# ✅ シミュレーション環境のセットアップ
config = Config()
loader = TrajectoryLoader()
car = simulation.CarSimulation()
run = simulation.RunSimulation()
visualizer = Visualization()
# optimization = Optimizer()
controller = TrajectoryController(mode="CTE")
predict = PredictCompute()
dubin = DubinsPath()

# ✅ 目標軌道をロード
trajectory = loader.sine_wave()
# trajectory = loader.arc(scale=5)
num_steps = 300

state_history, cte_history, steer_history, velocity_history = run.simulation(controller, predict, trajectory, dubin, dt=0.1, max_steps=num_steps)

visualizer.plot_simulation(trajectory, state_history)