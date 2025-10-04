from config import Config
import numpy as np
from physics import Physics
import time

class CarSimulation:
    def __init__(self):
        config = Config()
        params = config.get_vehicle_params()

        self.wheel_base = params["wheel_base"]  # 車輪間距離
        self.wheel_radius = params["wheel_radius"]  # タイヤ半径
        self.dt = params["dt"]  # 制御時間間隔
        self.st = params["st"]  # シミュレーション時間間隔

        self.v_max, self.v_min = 0.28, 0.07

        # 初期状態
        self.x = 0.0
        self.y = 0.0
        self.omega = 0.0
        self.theta = 0.0
        self.v = 0.0
        self.last_delta = 0.0
        self.delta = 0.0
        self.last_accel = 0.0
        self.accel = 0.0

    def initial_state(self, x, y, theta, omega, v):
        self.x = x
        self.y = y
        self.omega = omega
        self.theta = theta
        self.v = v

    def step(self, v_r, v_l):
        """左右の車輪速度から車両の挙動を更新"""
        self.v = (v_r + v_l) / 2.0  # 両輪の平均速度
        if abs(v_r - v_l) > 1e-4:
            R = self.wheel_base * 2 * (v_r + v_l) / (v_r - v_l)
            self.omega = self.v / R * self.dt
            L = 2 * R * np.sin(self.omega * np.pi / 2)
        else:
            self.omega = 0
            L = self.v * self.dt
        next_delta = self.theta + self.omega / 2
        
        self.x += L * np.cos(next_delta * np.pi)
        self.y += L * np.sin(next_delta * np.pi)

        self.theta += self.omega
        
        return self.x, self.y, self.theta, self.omega, self.v
    
    def step_theta(self, delta, accel):
        self.last_delta = delta
        if abs(self.last_delta-self.delta) > 1E-6:
            self.delta+=(self.last_delta-self.delta) * (1 - np.exp(-1*self.dt/self.st))
        else:
            self.delta = self.last_delta
        self.last_accel = accel
        if abs(self.last_accel-self.accel) > 1E-6:
            self.accel+=(self.last_accel-self.accel) * (1 - np.exp(-1*self.dt/self.st))
        else:
            self.accel = self.last_accel
        self.v = self.accel
        self.v = max(self.v_min, min(self.v_max, self.v))
        steer = self.delta * 2
        if abs(steer) > 1e-4:
            R = self.wheel_base / np.tan(steer)  # カーブの半径
            self.omega = steer * self.v * self.dt
            L = 2 * R * np.sin(self.omega / 2)
        else:
            self.omega = 0
            L = self.v * self.dt
        # next_delta = self.theta + np.pi + self.omega / 2
        next_delta = self.theta + self.omega / 2

        self.x += L * np.cos(next_delta)
        self.y += L * np.sin(next_delta)

        self.theta += self.omega

        return self.x, self.y, self.theta, self.omega, self.v, self.accel
    
    def get_state(self):
        """ 現在の車両状態を取得 """
        return self.x, self.y, self.theta, self.omega, self.v, self.accel
    
    def get_state_speed(self):
        """ 現在の車両状態を取得 """
        return self.x, self.y, self.theta, self.v 
    
    def simulate_wind_effect(v, theta, wind_speed, wind_angle):
        """風の影響を考慮した速度と角度の変化"""
        air_resistance = 0.1 * v**2  # 空気抵抗
        wind_force = 0.2 * wind_speed * np.cos(theta - wind_angle)  # 風の力
        lateral_force = 0.1 * wind_speed * np.sin(theta - wind_angle)  # 横風による影響
        
        v_new = v - air_resistance + wind_force
        theta_new = theta + lateral_force
        
        return v_new, theta_new
    
    def simulate_tire_friction(v, delta, friction_coefficient):
        """摩擦係数を考慮した車両の制動"""
        max_accel = friction_coefficient * 9.81  # 最大加速度 (F=μmg)
        
        # 摩擦が低いほどステアリングが効きにくい
        delta_new = delta * friction_coefficient  
        
        # 速度の変化（摩擦が低いと減速しやすい）
        v_new = max(0, v - (1 - friction_coefficient) * 0.05)
        
        return v_new, delta_new
    
    def detect_obstacle(x, y, obstacles):
        """障害物が近いかを判定"""
        for ox, oy in obstacles:
            if np.hypot(x - ox, y - oy) < 1.0:
                return True
        return False

class RunSimulation:
    def __init__(self):
        self.physics = Physics()
        
    def simulation(self, controller, predict, trajectory, dubin, dt=0.1, max_steps=1000):
        state_history = []  # 軌跡を記録
        cte_history = []  # 横方向誤差を記録
        steer_history = []  # ステアリング履歴
        velocity_history = []  # 速度履歴

        simulation = CarSimulation()
        trajectory_step = 0  # 描画用のインデックス
        target_step = 0 # lookahead_pointのインデックス
        threshold_flag = True
        delta, v = 0.0, 1.0
        ref_step = 0

        # 初期状態の設定
        theta_ref = controller.compute_initial_heading(trajectory)  # 目標の初期角度
        state = simulation.initial_state(0.0, 0.0, 0.0, 0.0, 0.0)  # 初期向きを合わせる
        ref_state = simulation.get_state()
        x, y, theta, omega, v, accel = ref_state
        control = {"delta": [0.0], "accel": [0.0]}
        pos_err, theta_err, cte, omega_err, now_index=predict.compute_error(simulation.get_state(), trajectory, x, y)
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
            x, y, theta, omega, v, accel = simulation.get_state()  # ✅ 現在の位置を取得
            state = [x, y, theta, omega, v, accel]
            
            x_reference, y_reference = trajectory[trajectory_step]

            # ✅ Pure Pursuit + PID + 最適化（GA / PSO）で制御値を計算
            #"""
            if threshold_flag:
                control, ref_state, ref_step = dubin.dubins_compute(state, trajectory, step, ref_state, ref_step, control)
            else:
                control, ref_state, ref_step = predict.adaptive_compute(state, trajectory, step, ref_state, ref_step, control)
            #"""
            #control, ref_state, ref_step = predict.adaptive_compute(state, trajectory, step, ref_state, ref_step, control)
            delta = control["delta"][0]
            accel = control["accel"][0]

            x_target, y_target = trajectory[target_step]

            # ✅ シミュレーション更新
            simulation.step_theta(delta, accel)
            x, y, theta, omega, v, accel = simulation.get_state()
            states = [x, y, theta, omega, v, accel]

            pos_err, theta_err, cte, omega_err, now_index=predict.compute_error(states, trajectory, x, y)
            #pos_err, theta_err = self.physics.compute_errors((x, y, theta), (x_target, y_target))
            
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