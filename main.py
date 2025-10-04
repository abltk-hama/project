import numpy as np
import time
from config.config import Config
from sim.simulation import Simulation
from sim.visualization import Visualization
from trajectory.trajectory_loader import TrajectoryLoader
from controllers.trajectory_control import TrajectoryController
from simple_predict import PredictCompute
from dubin_control import DubinsPath

# ✅ シミュレーション環境のセットアップ
config = Config()
loader = TrajectoryLoader()
car = Simulation.CarSimulation()
run = Simulation.RunSimulation()
visualizer = Visualization()
controller = TrajectoryController(mode="CTE")
predict = PredictCompute()
dubin = DubinsPath()

# ✅ 目標軌道をロード
trajectory = loader.sine_wave()
# trajectory = loader.arc(scale=5)
num_steps = 300

state_history, cte_history, steer_history, velocity_history = run.simulation(controller, predict, trajectory, dubin, dt=0.1, max_steps=num_steps)

visualizer.plot_simulation(trajectory, state_history)