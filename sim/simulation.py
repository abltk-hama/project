from config import Config
import numpy as np
from models.physics import Physics
from trajectory.geometry import geometry
from controllers.dubin_control import DubinsPath
from controllers.pso_adaptive import PSO_adaptive
import time

class RunSimulation:
    def __init__(self):
        self.physics = Physics()
        self.geometry = geometry()
        self.dubins = DubinsPath()
        self.pso = PSO_adaptive()
        
    def simulation(self, trajectory, dt=0.1, max_steps=1000):
        state_history = []  # 軌跡を記録
        cte_history = []  # 横方向誤差を記録
        steer_history = []  # ステアリング履歴
        velocity_history = []  # 速度履歴

        simulation = self.physics
        trajectory_step = 0  # 描画用のインデックス
        target_step = 0 # lookahead_pointのインデックス
        threshold_flag = True
        delta, v = 0.0, 1.0
        ref_step = 0

        # 初期状態の設定
        theta_ref = self.geometry.get_path_theta(trajectory, 0)
        state = self.physics.initial_state(0.0, 0.0, 0.0, 0.0, 0.0)  # 初期向きを合わせる
        ref_state = self.physics.get_state()
        x, y, theta, omega, v = ref_state
        control = {"delta": [0.0], "accel": [0.0]}
        cte = self.geometry.compute_cte(x, y, theta, trajectory, 0)
        theta_err = self.geometry.compute_heading_error(state, trajectory, 0)
        if abs(cte) > 2.0 or abs(theta_err) > np.radians(30):
            threshold_flag = True
        else:
            threshold_flag = False

        # ✅ ログ保存
        state_history.append([simulation.x, simulation.y])
        steer_history.append(0.0)
        velocity_history.append(0.0)
        cte_history.append(0.0)
         
        for step in range(max_steps):
            x, y, theta, omega, v = self.physics.get_state()  # ✅ 現在の位置を取得
            state = [x, y, theta, omega, v]
            
            x_reference, y_reference = trajectory[trajectory_step]

            # ✅ Pure Pursuit + PID + 最適化（GA / PSO）で制御値を計算
            #"""
            if threshold_flag:
                control, ref_state, ref_step = self.dubins.dubins_compute(state, trajectory, step, ref_state, ref_step, control)
            else:
                control, ref_state, ref_step = self.pso.adaptive_compute(state, trajectory, step, ref_state, ref_step, control)
            #"""
            #control, ref_state, ref_step = predict.adaptive_compute(state, trajectory, step, ref_state, ref_step, control)
            delta = control["delta"][0]
            accel = control["accel"][0]

            x_target, y_target = trajectory[target_step]

            # ✅ シミュレーション更新
            simulation.step_theta(delta, accel)
            x, y, theta, omega, v = self.physics.get_state()
            state = [x, y, theta, omega, v]
            _, _, now_index = self.geometry.find_closest_point(x, y, trajectory)

            theta_err = self.geometry.compute_heading_error(state, trajectory, now_index)
            cte = self.geometry.compute_cte(x, y, theta, trajectory, now_index)
            path = self.geometry.get_path_data(state, trajectory, N=5)
            omega_err = self.geometry.compute_omega_error(state, path)
            
            target_step = now_index
            print(f"Step:{trajectory_step:.1f},car=(x:{x:.3f},y:{y:.3f}),cte={cte:.3f}, theta_err={theta_err:.3f}, delta={omega:.4f}, v={v:.4f})")

            # ✅ ログ保存
            state_history.append([simulation.x, simulation.y])
            steer_history.append(delta)
            velocity_history.append(accel)
            cte_history.append(cte)

            time.sleep(0.02)  # ✅ シミュレーションの時間調整

            if abs(cte) > 2.0 or abs(omega_err) > np.radians(30):
                threshold_flag = True

            if threshold_flag:
                if abs(cte) < 0.1 and abs(omega_err) < np.radians(10):
                    print("車両が目標軌道に復帰しました。")
                    threshold_flag = False

            if trajectory_step == 0:
                trajectory_step = 1
            if trajectory_step <= target_step and trajectory_step <= len(trajectory):
                if not threshold_flag:
                    trajectory_step += 1
            
            if trajectory_step >= len(trajectory)-1:
                break
            """
            if distance_to_target > simulation_stop_distance:
                print("車両が離れすぎました。")
                break
            """
        
        print("シミュレーション終了",step)
        return state_history, cte_history, steer_history, velocity_history