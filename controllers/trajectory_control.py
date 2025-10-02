import numpy as np
from physics import Physics
from lqr import LQRController
from config import Config
import random

class TrajectoryController:
    def __init__(self, optimize_interval=50, mode="PID", dt=0.1):
        config = Config()
        params = config.get_vehicle_params()

        self.mode = mode
        self.dt = params["dt"]
        self.wheel_base = params["wheel_base"]
        self.k_cte = 0.005
        self.time_lag_opt, self.speed, self.steer = 0.85, 0.0, 0.0
        self.physics = Physics()
        self.lookahead_distance, self.v_max, self.v_min = 3.0, 1.0, 0.1
        self.v_max, self.v_min, self.theta_max, self.theta_min = 1.0, 0.001, 0.25, -0.25
        self.k_p_v, self.k_d_v, self.k_i_v, self.k_c_v = 0.5, 0.1, 0.05, 0.1
        self.k_p_theta, self.k_d_theta, self.k_i_theta, self.k_c_theta = 0.5, 0.1, 0.05, 0.01
        self.k_p_cte, self.k_d_cte, self.k_i_cte = 0.08, 0.1, 0.00002
        self.prev_cross_track_error, self.cross_track_error_sum, self.prev_cte = 0.0, 0.0, 0.0
        self.prev_theta_error, self.theta_error_sum, self.cte_error_sum = 0.0, 0.0, 0.0
        self.steer_step, self.accel_step, self.prev_steer, self.prev_v = 0.0, 0.0, 0.0, 0.0
        self.optimize_interval = optimize_interval  # ✅ 最適化の間隔（50回に1回実行）
        self.step_count = 0  # ✅ ループ回数をカウント

    def compute_control(self,simulation, trajectory, trajectory_step, threshold_flag, v, delta):
        """最適化の実行"""
        if self.mode == "PID":
            return self._pid_control(simulation, trajectory, trajectory_step, threshold_flag, v, delta)
        elif self.mode == "LQR":
            return self._lqr_control(simulation, trajectory, trajectory_step, threshold_flag, v, delta)
        elif self.mode == "CTE":
            return self.compute_steering_angle(simulation, trajectory)
        else:
            raise ValueError("最適化手法が不明です。'GA' / 'PSO' / 'GD' のいずれかを選択してください。")

    def update_params(self, params):
        """最適化結果を適用"""
        #theta:Kp, Ki, Kd,cte:Kp, Ki,  Kd,delta;min,max,v:min,max
        self.k_p_theta, self.k_d_theta, self.k_i_theta, self.k_p_cte, self.k_d_cte, self.k_i_cte, self.theta_min, self.theta_min, self.v_min, self.v_max = params

    def update_pidparams(self, params):
        """最適化結果を適用"""
        #theta:Kp, Ki, Kd,cte:Kp, Ki,  Kd,delta;min,max,v:min,max
        self.k_p_theta, self.k_d_theta, self.k_i_theta, self.k_p_cte, self.k_d_cte, self.k_i_cte = params

    def _pid_control(self, simulation, trajectory, trajectory_step, threshold_flag, v, delta):
        """Pure Pursuit + 相互作用を考慮した PID 制御"""
        x, y, theta = simulation.get_state()
        state = (x, y, theta)

        # ① 目標点 (`lookahead_point`) の取得
        if threshold_flag :
            target_x, target_y, trajectory_step = self.physics.find_target_point(state, trajectory, trajectory_step, self.lookahead_distance)
        else:
            target_x, target_y = trajectory[trajectory_step]
        lookahead_point = (target_x, target_y)

        # ② 軌道との最短距離誤差 (`cross_track_error`) を計算
        x_closest, y_closest, closest_step = self.physics.find_closest_point(x, y, trajectory)
        cross_track_error = np.linalg.norm([x_closest - x, y_closest - y])

        # ③ `compute_curvature()` を利用し、ターゲット角度 (`theta_target`) を取得
        curvature, theta_target = self.physics.compute_curvature(trajectory, trajectory_step)

        # ④ 角度誤差 (`theta_error`) の修正
        theta_error = theta_target - theta  # ✅ 軌道上の角度との差分

        # ⑥ PID 制御 (相互作用あり)
        control_steer = (
            self.k_p_theta * theta_error +
            self.k_d_theta * (theta_error - self.prev_theta_error) +
            self.k_c_theta * cross_track_error +   # ✅ 横方向誤差も影響を与える
            self.k_d_theta * self.theta_error_sum
        )

        control_speed = (
            self.k_p_v * cross_track_error +
            self.k_d_v * (cross_track_error - self.prev_cross_track_error) +
            self.k_c_v * abs(theta_error) + # ✅ 角度誤差が大きいほど減速
            self.k_d_v * self.cross_track_error_sum
        )

        # ⑦ 状態更新
        self.prev_theta_error = theta_error
        self.prev_cross_track_error = cross_track_error

        control_steer = max(min(self.theta_max,theta_target), self.theta_min)

        # ✅ デバッグログ
        print(f"Step {closest_step}:car=({x:.2f}, {y:.2f}), target=({target_x:.2f}, {target_y:.2f}),"
            f"cross_track_err={cross_track_error:.4f}, theta_err={theta_error:.4f}, "
            f"v={control_speed:.4f}, delta={control_steer:.4f}")

        return control_steer, control_speed, lookahead_point, trajectory_step
    
    """
    LQR制御
    """

    def _lqr_control(self, simulation, trajectory, trajectory_step, threshold_flag, v, delta):
        control_steer: float
        # 初期状態 [x, y, theta, v]
        x, y, theta = simulation.get_state()
        state_data = (x, y, theta)
        state = np.array([x, y, theta, v])

        self.prev_v = v
        self.prev_steer = delta

        # ① 目標点 (`lookahead_point`) の取得
        if threshold_flag :
            target_x, target_y, trajectory_step = self.physics.find_target_point(state_data, trajectory, trajectory_step, self.lookahead_distance)
        else:
            target_x, target_y = trajectory[trajectory_step]
        lookahead_point = (target_x, target_y)

        # ② 軌道との最短距離誤差 (`cross_track_error`) を計算
        x_closest, y_closest, closest_step = self.physics.find_closest_point(x, y, trajectory)
        cross_track_error = np.linalg.norm([x_closest - x, y_closest - y])

        # ③ `compute_curvature()` を利用し、ターゲット角度 (`theta_target`) を取得
        if trajectory_step==len(trajectory):
            trajectory_step=len(trajectory) - 2
        curvature, theta_target = self.physics.compute_curvature(trajectory, trajectory_step)

        # 目標状態 [x_ref, y_ref, theta_ref, v_ref]
        ref_state = np.array([target_x, target_y, theta_target, v])  

        # LQR モデルの作成
        state_transition = StateTransition(dt=0.1)
        get_state = state_transition.get_state_matrices(*state)
        lqr = LQRController()

        # 制御入力を計算
        control = lqr.compute_control(state, ref_state, get_state)
        control_steer, control_speed = control
        control_steer = max(min(self.theta_max,control_steer), self.theta_min)
        control_speed = max(min(control_speed, self.v_max), self.v_min)
        self.prev_steer = control_steer
        self.prev_v = control_speed

        # ✅ デバッグログ
        # print(f"Step {closest_step}:car=({x:.2f}, {y:.2f}), target=({target_x:.2f}, {target_y:.2f}),"
        #    f"v={control_speed:.4f}, delta={control_steer:.4f}")

        return control_steer, control_speed, lookahead_point, trajectory_step
    
    """
    CTE制御
    """
    
    def compute_cte(self, state, trajectory):
        """
        CTE（Cross-Track Error）を計算
        state: 現在の状態 (x, y, theta, v)
        trajectory: 目標経路（リスト）
        """
        # 1. 現在位置に最も近い目標点を探す
        closest_index = np.argmin(np.linalg.norm(trajectory - state[:2], axis=1))
        closest_point = trajectory[closest_index]

        # 2. 経路の角度（目標点から次の点への方向）
        if closest_index < len(trajectory) - 1:
            path_theta = np.arctan2(trajectory[closest_index + 1][1] - closest_point[1],
                                    trajectory[closest_index + 1][0] - closest_point[0])
        else:
            path_theta = state[2]  # 終点なら現在の角度

        # 3. CTE の計算
        dx = state[0] - closest_point[0]
        dy = state[1] - closest_point[1]
        cte = dy * np.cos(path_theta) - dx * np.sin(path_theta)

        return cte, closest_index
    
    def compute_steering_angle(self, state, trajectory):
        """
        Stanley Control for path tracking
        """
        x, y, theta, _, v = state.get_state()
        set_state = (x, y, theta, v)
        # 角度誤差を計算
        closest_index = np.argmin(np.linalg.norm(trajectory - set_state[:2], axis=1))
        path_theta = np.arctan2(trajectory[closest_index + 1][1] - trajectory[closest_index][1],
                                trajectory[closest_index + 1][0] - trajectory[closest_index][0])
        theta_e = path_theta - set_state[2]

        # CTE の計算
        cte, closest_index = self.compute_cte(set_state, trajectory)

        target_x, target_y = trajectory[closest_index]

        # Stanley 制御
        steer_angle = theta_e + np.arctan(self.k_cte * cte / (set_state[3] + 1e-3))

        self.cte_error_sum += cte  # 積分
        cte_error_diff = cte - self.prev_cte  # 微分
        self.prev_cte = cte

        errors = [cte, self.cte_error_sum, cte_error_diff]

        # self.update_pidparams(self.select_best_pid(set_state, self, trajectory, errors, 20))

        steering = self.k_p_cte * cte + self.k_i_cte * self.cte_error_sum + self.k_d_cte * cte_error_diff
        steering = max(min(self.theta_max,steering), self.theta_min)
        v = self.k_c_v / (theta_e + 0.1)
        v = max(min(self.v_max,v), self.v_min)

        # ✅ デバッグログ
        print(f"Step {closest_index}:car=({x:.2f}, {y:.2f}), target=({target_x:.2f}, {target_y:.2f}),"
           f"v={v:.4f}, delta={steering:.4f}, CTE={cte}")

        return -steering, v, trajectory[closest_index], closest_index
    
    def compute_initial_heading(self, trajectory):
        """ 経路の最初の向きを計算 """
        x1, y1 = trajectory[0]
        x2, y2 = trajectory[1]
        theta_ref = np.arctan2(y2 - y1, x2 - x1)  # 経路の最初の角度
        return theta_ref
    
    def predict_future_trajectory(self, state, controller, errors, pid_params, N):
        """
        指定されたPIDパラメータで未来Nステップの軌道を予測
        """
        temp_state = np.copy(state)
        predicted_trajectory = []

        for _ in range(N):
            # 制御計算
            delta, v = controller._compute_control(pid_params, errors)
            # 状態遷移の更新（簡易版）
            # ✅ 車両の運動モデル（対向2輪）
            v_r = v * np.sin((delta + 0.25)*np.pi)  # 右車輪速度
            v_l = v * np.cos((delta + 0.25)*np.pi)  # 左車輪速度
            temp_state = self.predict_state(temp_state, v_l, v_r, 1)
            predicted_trajectory.append(temp_state)

        return np.array(predicted_trajectory)
    
    def select_best_pid(self, state, controller, trajectory, errors, N, num_samples=5):
        """
        複数のPIDパラメータセットを試し、最も軌道に近づくものを選択
        """
        best_pid_params = None
        best_distance = float('inf')

        # ランダムにPIDパラメータを生成（最適化範囲は調整）
        pid_candidates = self.generate_random_pid_params(num_samples)

        for pid_params in pid_candidates:
            predicted_traj = self.predict_future_trajectory(state, controller, errors, pid_params, N)
            
            # 軌道との最短距離を評価
            distance = self.compute_min_distance(predicted_traj, trajectory)
            distance_mean = np.mean(np.abs(distance))
            
            if distance_mean < best_distance:
                best_distance = distance_mean
                best_pid_params = pid_params

        return best_pid_params
    
    def predict_state(self, state, v_r, v_l, N):
        """左右の車輪速度から車両の挙動を更新"""
        x, y, theta, v = state
        v = (v_r + v_l) / 2.0  # 両輪の平均速度
        omega = (v_r - v_l) / self.wheel_base  # 角速度
        
        x += v * np.cos(theta) * self.dt * N
        y += v * np.sin(theta) * self.dt * N
        theta += omega * self.dt
        
        return x, y, theta, v
    
    def generate_random_pid_params(self, num_samples):
        pid_params = []
        pid_param = [self.k_p_cte, self.k_i_cte, self.k_d_cte, self.k_p_v, self.k_i_v, self.k_d_v]
        pid_params.append(pid_param)
        for i in range(num_samples):
            Kp_cte = random.uniform(self.k_p_cte-self.k_p_cte/100, self.k_p_cte+self.k_p_cte/1000)
            Ki_cte = random.uniform(self.k_i_cte-self.k_i_cte/1000, self.k_i_cte+self.k_i_cte/1000)
            Kd_cte = random.uniform(self.k_d_cte-self.k_d_cte/1000, self.k_d_cte+self.k_d_cte/1000)
            Kp_v = random.uniform(self.k_p_v-self.k_p_v/1000, self.k_p_v+self.k_p_v/1000)
            Ki_v = random.uniform(self.k_i_v-self.k_i_v/1000, self.k_i_v+self.k_i_v/1000)
            Kd_v = random.uniform(self.k_d_v-self.k_d_v/1000, self.k_d_v+self.k_d_v/1000)
            pid_param = [Kp_cte, Ki_cte, Kd_cte, Kp_v, Ki_v, Kd_v]
            pid_params.append(pid_param)
        return pid_params

    def _compute_control(self, pid_params, errors):
        err, err_sum, err_diff = errors
        Kp_theta, Ki_theta, Kd_theta, Kp_v, Ki_v, Kd_v = pid_params
        steering = Kp_theta * err + Ki_theta * err_sum + Kd_theta * err_diff
        steering = max(min(self.theta_max,steering), self.theta_min)
        v = Kp_v * err + Ki_v * err_sum + Kd_v * err_diff
        return steering, v
    
    def compute_min_distance(self, predicted_traj, trajectory):
        """
        CTE（Cross-Track Error）を計算
        state: 現在の状態 (x, y, theta, v)
        trajectory: 目標経路（リスト）
        """
        for i in range(len(predicted_traj)):
            # 1. 現在位置に最も近い目標点を探す
            state = predicted_traj[i]
            closest_index = np.argmin(np.linalg.norm(trajectory - state[:2], axis=1))
            closest_point = trajectory[closest_index]

            # 2. 経路の角度（目標点から次の点への方向）
            if closest_index < len(trajectory) - 1:
                path_theta = np.arctan2(trajectory[closest_index + 1][1] - closest_point[1],
                                        trajectory[closest_index + 1][0] - closest_point[0])
            else:
                path_theta = state[2]  # 終点なら現在の角度

            # 3. CTE の計算
            dx = state[0] - closest_point[0]
            dy = state[1] - closest_point[1]
            cte = dy * np.cos(path_theta) - dx * np.sin(path_theta)

        return cte, closest_index