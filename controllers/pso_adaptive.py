import numpy as np
from config.config import Config
from trajectory.geometry import geometry
import matplotlib.pyplot as plt
import random
import time

class PSO_adaptive():
    def __init__(self):
        config = Config()
        params = config.get_vehicle_params()
        self.geometry = geometry()

        # シミュレーションの単位時間を読み込み
        self.dt = params["dt"]
        self.pt = params["pt"]
        # 車両設定を読み込み
        self.wheel_base = params["wheel_base"] 
        self.wheel_radius = params["wheel_radius"] 

        # Qテーブル
        self.num_states = 8000
        self.num_actions = 15
        self.Q_table = np.zeros((self.num_states, self.num_actions))

        # 予測制御で使うエラー値を定義
        # 1.瞬間残差のリスト => 予測した制御との誤差を記録
        self.residual_collections = {"cte": [], "theta": []}
        # 2.平均残差 => 瞬間誤差を平均して記録 => 外乱成分、機械特性への耐性
        self.residual_average = {"cte": 0.0, "theta": 0.0}
        # 3.ひとつ前のエラー蓄積量 => 予測した状態と現在の状態との誤差の蓄積量
        self.prev_accumulated_error = {"cte": 0.0, "theta": 0.0}
        self.accumulated_errors = []

        self.prev_predict_error = {"cte": 0.0, "theta": 0.0}
        self.predict_err_gain = {"cte": 1.0, "theta": 1.0}
        self.theta_trend_gain = {"cte": 0.0, "theta": 0.0}

        # 係数リスト
        # ae(accumulated_error): エラー蓄積量
        # ir(instant_residual): 直前のステップでの予測値と実測値の差分(残差)
        # ir(mean_residual): 測値と実測値の差分の平均(残差)
        # ハンドル補正に対する係数
        self.K_delta = {
            "cte":{
                "ae": 0.0,
                "ir": 0.0,
                "mr": 0.0
            },
            "theta":{
                "ae": 0.0,
                "ir": 0.0,
                "mr": 0.0
            }
        }
        # アクセル補正に対する係数
        self.K_accel = {
            "cte":{
                "ae": 0.0,
                "ir": 0.0,
                "mr": 0.0
            },
            "theta":{
                "ae": 0.0,
                "ir": 0.0,
                "mr": 0.0
            }
        }

        # 一度に効くステアリング/アクセルの制限
        self.acc_limit, self.steer_limit = 0.2, 0.24

        # predict_state用
        self.last_delta, self.delta, self.last_accel, self.accel = 0.0, 0.0, 0.0, 0.0
        self.v_max, self.v_min = 0.28, 0.07
        self.deg = 20
        self.theta_max, self.theta_min = self.deg/180*np.pi, -self.deg/180*np.pi

        # pso用
        self.opt = False
        self.q12, self.q13, self.q14, self.q15 = 0.0, 0.0, 0.0, 0.0
        self.q23, self.q24, self.q25, self.q34, self.q35, self.q45 = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
    
    def predict_state(self, state, delta, accel, N):
        x, y, theta, ref_delta, ref_v = state
        omega = ref_delta
        steer = omega
        last_delta = delta
        pred_delta = ref_delta
        last_accel = accel
        pred_accel = ref_accel
        for i in range(N):
            pred_delta+=(last_delta-pred_delta) * np.exp(-1*self.pt/self.dt)
            pred_accel+=(last_accel-pred_accel) * np.exp(-1*self.pt/self.dt)
            ref_v = pred_accel
            ref_v = max(self.v_min, min(self.v_max, ref_v))
            steer = pred_delta/2

            if abs(steer) > 1e-4:
                R = self.wheel_base / np.tan(steer)  # カーブの半径
                omega = ref_v / R * self.dt
                L = 2 * R * np.sin(omega / 2)
            else:
                omega = 0
                L = ref_v * self.dt
            next_delta = theta + np.pi + omega / 2

            x += -L * np.cos(next_delta)
            y += -L * np.sin(next_delta)

            theta += omega

        return x, y, theta, pred_delta, ref_v
    
    def adaptive_compute(self, state, reference_trajectory, current_step, ref_state, ref_step, control, N=5, k=2):
        """
        適応型制御
        状態ベクトルを元に、PSOで最適な制御パラメータを探索し、制御を行う。
        モードによって、異なる状態ベクトルを用いて制御を行う。
        モード1: 経路追従モード
          状態ベクトル: [CTE, ω誤差, 速度]
        モード2: 目標地点モード
            状態ベクトル: [目標までの距離, 目標方向誤差, 速度]
        """
        # --- 設定 ---
        N_PARTICLES = 2      # 粒子数
        N_STEPS = 9          # 仮走行ステップ数
        n_kq = 10           # 制御パラメータ数
        core_range = 0.1
        alpha = 0.8
        beta = 0.01
        gamma = 0.9
        eta = 0.04
        n_obs = 6
        np.random.seed(42)    # 再現性のため固定
        k_v = 1.0
        n_iter = 20
        w, c1, c2 = 0.5, 1.3, 1.3  # PSOパラメータ
        v = np.zeros((N_PARTICLES, n_kq))  # 速度初期
        all_trace_loss = []
        graph = False

        now_x, now_y, now_theta, now_delta, now_v = state

        # --- 制御式 ---
        def base_omega(cte, omega_err, theta, v, kq):
            q12, q13, q14, q15, q23, q24, q25, q34, q35, q45 = kq
            x = np.array([cte, omega_err, theta, v, 1.0])
            Q = np.array([
                [0.0, q12, q13, q14, q15],
                [q12, 0.0, q23, q24, q25],
                [q13, q23, 0.0, q34, q35],
                [q14, q24, q34, 0.0, q45],
                [q15, q25, q35, q45, 0.0]
            ])

            return x.T @ Q @ x
        
        # --- 評価関数（誤差² + ハンドル変動²） ---
        def evaluate_cost(cte_log, theta_log, omega_log, steer_log, gamma = 0.9):
            ave_cte = calculate_mean(cte_log, gamma)
            trend_cte = calculate_diff(cte_log)
            ave_theta = calculate_mean(theta_log, gamma)
            trend_theta = calculate_diff(theta_log)
            ave_omega = calculate_mean(omega_log, gamma)
            trend_omega = calculate_diff(omega_log)
            return ave_cte/2 + ave_theta/2 + ave_omega/2 + trend_cte[0]*2 + trend_theta[0]*2 + trend_omega[0]*2
        
        def calculate_diff(log):
            trend = []
            # 先行して修正された重み付き平均を前提
            for i in range(1, len(log)):
                d_mag = abs(log[i]) - abs(log[i-1])
                trend += max(0.0, d_mag)   # 悪化のみ罰則
            return trend
        
        def calculate_mean(log, gamma=0.9):
            mean = 0.0
            weight = gamma ** np.arange(len(log))
            weigts = np.sum(weight)
            for i in range(1, len(log)-1):
                mean += weight[i] * abs(log[i])
            mean /= weigts
            return mean
        
        # --- 仮走行を1粒子ぶんシミュレーション ---
        def simulate_particle(state, kq):
            q12, q13, q14, q15, q23, q24, q25, q34, q35, q45 = kq
            cte = self.geometry.compute_cte(state, reference_trajectory)
            theta_err = self.geometry.compute_heading_error(state, reference_trajectory)
            path = self.geometry.get_path_data(state, reference_trajectory, N=5)
            omega_err = self.geometry.compute_omega_error(state, path)
            base_state = state
            cte_log = [cte]  # 初期誤差
            theta_log = [theta_err]  # 初期角度誤差（10°）
            omega_log = [omega_err]
            steer_log = [now_delta]
            before_cte, before_omega, before_theta = cte, omega_err, theta_err
            brefore_v = now_v

            for t in range(N_STEPS):
                delta = base_omega(before_cte, before_omega, before_theta, brefore_v, kq)
                delta = self.theta_max * np.tanh(delta/self.theta_max/3)  # ハンドル制限
                now_accel = k_v * np.cos(delta)
                pred_state = self.predict_state(base_state, delta, now_accel, N=1)
                pred_x, pred_y, pred_theta, pred_delta, pred_v = pred_state
                cte = self.geometry.compute_cte(pred_state, reference_trajectory)
                theta_err = self.geometry.compute_heading_error(pred_state, reference_trajectory)
                path = self.geometry.get_path_data(pred_state, reference_trajectory, N=5)
                omega_err = self.geometry.compute_omega_error(pred_state, path)
                before_cte, before_omega, before_theta = cte, omega_err, theta_err
                brefore_v = pred_v

                # 仮の誤差変化モデル（本来は環境に依存）
                cte_log.append(cte)
                theta_log.append(theta_err)
                omega_log.append(omega_err)
                steer_log.append(pred_delta)
                base_state = pred_state

            return evaluate_cost(cte_log, theta_log, omega_log, steer_log, gamma), (cte_log, theta_log, omega_log, steer_log)

        # --- 粒子群の実行 ---
        particles = np.random.uniform(low=-0.5, high=0.5, size=(N_PARTICLES, n_kq))
        m_F = np.zeros((N_PARTICLES, n_kq))
        F = np.zeros((N_PARTICLES, n_kq))
        results = []
        p_best = []
        pbest_p = particles
        trace_loss = []
        if self.opt == True:
            particles[0] = (self.q12, self.q13, self.q14, self.q15, self.q23, self.q24, self.q25, self.q34, self.q35, self.q45)

        for it in range(n_iter):
            for i in range(N_PARTICLES):
                cost, logs = simulate_particle(state, particles[i])
                results.append((cost, particles[i], logs))
                fn = np.zeros((n_obs, n_kq))
                for o in range(n_obs):
                    obs_pos = particles[i] + np.random.uniform(-core_range, core_range, size=n_kq)
                    obs_cost, logs = simulate_particle(state, obs_pos)
                    fn[o] = (particles[i] - obs_pos) * np.sign(obs_cost - cost)
                F[i] = np.mean(fn, axis=0)
                m_F[i] = beta * m_F[i] + (1 - beta) * F[i]
                if it == 0:
                    p_best.append((cost, particles[i], logs))
                elif it > 0 and cost < p_best[i][0]:
                    p_best[i] = (cost, particles[i], logs)
                    pbest_p = particles[i]

            # --- 最良粒子の表示 ---
            results.sort(key=lambda x: x[0])
            best = results[0]
            results.clear()
            best_cost, (q12, q13, q14, q15, q23, q24, q25, q34, q35, q45), (cte_log, theta_log, omega_log, steer_log) = best
            best_p = (q12, q13, q14, q15, q23, q24, q25, q34, q35, q45)
            r1, r2 = np.random.rand(), np.random.rand()
            #for i in range(N_PARTICLES):
            #    v[i] = w*v + c1*r1*(pbest_p-particles[i]) + c2*r2*(best_p-particles[i])
            #    particles[i] += v[i]
            v = w*v + c1*r1*(pbest_p-particles) + c2*r2*(best_p-particles) + alpha*F + eta*m_F
            particles += v
            trace_loss.append(best_cost)
        all_trace_loss.append(trace_loss)

        if graph == True:
            
            plt.figure(figsize=(8, 6))
            for i in range(len(all_trace_loss)):
                plt.plot(all_trace_loss[i], label=f"Iteration {i+1}")
            plt.legend()
            plt.ylabel("Y [m]")
            plt.title("Trajectory Tracking Simulation")
            plt.grid(True)
            plt.show()

        if self.opt == False:
            self.opt = True
        self.q12, self.q13, self.q14, self.q15, self.q23, self.q24, self.q25, self.q34, self.q35, self.q45 = best_p
        cte = self.geometry.compute_cte(state, reference_trajectory)
        theta_err = self.geometry.compute_heading_error(state, reference_trajectory)
        path = self.geometry.get_path_data(state, reference_trajectory, N=5)
        omega_err = self.geometry.compute_omega_error(state, path)
        omega = base_omega(cte, omega_err, theta_err, now_v, best_p)
        omega = self.theta_max * np.tanh(omega/self.theta_max/3)
        accel = k_v * np.cos(omega)
        print(f"最適化終了: {best_cost:.3f} / delta : {omega:.3f}")
        predict_control = {"delta": [], "accel": []}

        predict_control["delta"].append(omega)
        predict_control["accel"].append(accel)

        return predict_control, ref_state, ref_step