import numpy as np
from simulation import CarSimulation
import matplotlib.pyplot as plt

simulation = CarSimulation()
simulation.initial_state(0, 0, 0.0, 0.0, 0.0)
sim_state = simulation.get_state()

car_history=[[0,0]]
target=[1,1]
unit_omega=(np.arctan2((target[0]-sim_state[0]),(target[1]-sim_state[1]))-sim_state[2])
L=np.sqrt((target[0]-sim_state[0])**2+(target[1]-sim_state[1])**2)
target_R=L/2/np.sin(unit_omega)
delta=(np.arctan2((target[0]-sim_state[0])*0.2,(target[1]-sim_state[1]))-sim_state[2])
delta=delta/target_R
k=0
for i in range(10):
    for k in range(10):
        simulation.step_theta(delta, 1.0)
        sim_state = simulation.get_state()
        position=[sim_state[0],sim_state[1]]
        car_history.append(position)
    print(f"ステアリング(制御側:{np.degrees(delta):.1f}, 車両側:{np.degrees(sim_state[3]):.1f}), 車両角度: {np.degrees(sim_state[2]):.1f}")

state_x, state_y = zip(*car_history)

plt.figure(figsize=(8, 6))
plt.plot(state_x, state_y, 'r-', label="Actual Path")  # 追従した軌道
plt.legend()
plt.xlabel("X [m]")
plt.ylabel("Y [m]")
plt.title("Trajectory Tracking Simulation")
plt.grid(True)
plt.show()

