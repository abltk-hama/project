import numpy as np
from config import Config
from physics import Physics
import matplotlib.pyplot as plt
import random
import time

class PredictCompute():
    def __init__(self):
        config = Config()
        params = config.get_vehicle_params()
        self.physics = Physics()

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

    def predict_compute(self, state, reference_trajectory, current_step, ref_state, ref_step, control, N=5, k=2):
        """
        Nステップ後の状態を運動モデルを用いて予測する。

        Parameters:
            state: 現在の状態 [x, y, θ, ω, v]
            reference_trajectory: 目標経路
            current_step: 現在のstep
            ref_state: 予測された状態 [x, y, θ, ω, v]
            ref_step: 予測step
            control: 1つ先の予測制御 [delta, accel]
            N: 予測ステップ数 Nステップ単位で状態を予測する
            k: 制御適用の回数 Nステップごとに制御を適用して予測

        Returns:
            predict_control: 予測した制御 [delta, accel]
            ref_state: 予測した状態 [x, y, θ, ω, v]
            ref_step: 予測のstep
        """
        # 係数をセット

        # エラー蓄積量をリセット
        self.accumulated_errors.clear
        instant_residual = {"cte": 0.0, "theta": 0.0}

        ref_status = {"x": [], "y": [], "theta": []}

        # 現在の状態を取得
        x, y, theta, now_delta, v, accel = state
        ref_status["x"].append(x)
        ref_status["y"].append(y)
        ref_status["theta"].append(theta)

        # 予測された状態も取得(step: 0では無効)
        ref_x, ref_y, ref_theta, ref_delta, ref_v, ref_accel = ref_state
        ref_status["x"].append(ref_x)
        ref_status["y"].append(ref_y)
        ref_status["theta"].append(ref_theta)

        # 1つ以上制御が予測されていれば、更なるref_statusを取得
        if len(control["delta"]) > 1:
            for i in range(len(control)):
                predicted_state = self.predict_state(state, control["delta"][i], control["accel"][i], N)
                ref_x, ref_y, ref_theta, ref_delta, ref_v = predicted_state
                ref_status["x"].append(ref_x)
                ref_status["y"].append(ref_y)
                ref_status["theta"].append(ref_theta)

        ref_cte, ref_theta_err, ref_index = self.compute_error(state, reference_trajectory)

        # 現在のstepが予測されていたなら、現在の状態と予測された状態との誤差(残差)を取る　=> residual_error
        if current_step == ref_step:
            for i in range(len(ref_status["x"])-1):
                residual_cte = (ref_status["x"][i+1]-ref_status["x"][i])**2
                residual_cte += (ref_status["y"][i+1]-ref_status["y"][i])**2
                residual_cte = np.sqrt(residual_cte)
                instant_residual["cte"]=residual_cte
                residual_theta = (ref_theta - theta)
                instant_residual["theta"]=residual_theta
            self.residual_collections["cte"].append(instant_residual["cte"])
            self.residual_average["cte"] = np.mean(self.residual_collections["cte"])
            self.residual_collections["theta"].append(instant_residual["theta"])
            self.residual_average["theta"] = np.mean(self.residual_collections["theta"])
        else:
            instant_residual["cte"]=self.residual_collections["cte"][len(self.residual_collections["cte"])-1]
            instant_residual["theta"]=self.residual_collections["theta"][len(self.residual_collections["theta"])-1]

        predict_control = {"delta": [], "accel": []}

        best_control, ref_state = self._predict_compute(state, reference_trajectory, N)
        best_delta, best_accel = best_control
        # best_delta = best_control["delta"]
        # best_accel = best_control["accel"]
        
        if ref_step == 0 or ref_step == current_step:
            ref_step = N + current_step

        predict_delta = best_delta
        predict_accel = best_accel
        predict_control["delta"].append(predict_delta)
        predict_control["accel"].append(predict_accel)

        # print(f"BestControl:(delta: {predict_delta}, accel: {predict_accel})")
        # print(f"InstantResidual:{instant_residual}, MeanResidual:{self.residual_average}")

        return predict_control, ref_state, ref_step
    
    def predict_state(self, state, delta, accel, N):
        x, y, theta, ref_delta, ref_v, ref_accel = state
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

        return x, y, theta, pred_delta, ref_v, pred_accel
    
    def compute_error(self, state, reference_trajectory, target_x, target_y):
        """
        誤差計算
        pos_err: 位置誤差
        theta_err: 方位誤差(目標地点への向きと姿勢角の差)
        cte: 横方向誤差
        omega_err: スピン誤差(経路の曲率とハンドリングの差)
        """
        x_pred, y_pred, θ_pred, _, _, _ = state  # 予測状態

        pos_err = np.sqrt((x_pred - target_x) ** 2 + (y_pred - target_y) ** 2)  # 位置誤差
        x_ref, y_ref, ref_index = self.physics.find_forward_point(x_pred, y_pred, reference_trajectory)
        CTE = np.linalg.norm([x_pred - x_ref, y_pred - y_ref])  # 横方向誤差
        theta_err = self.compute_theta_error(np.arctan2(y_ref - y_pred, x_ref - x_pred), θ_pred)  # 方位誤差
        path_data = self.get_path_data(x_pred, y_pred, reference_trajectory, N=5)
        omega_err = self.compute_omega_error(state, path_data)  # スピン誤差

        return pos_err, theta_err, CTE, omega_err,  ref_index
    
    def predict_compute_error(self, state, reference_trajectory, N):
        ref_x, ref_y, ref_theta, ref_delta, ref_v, ref_accel = state
        path_list = self.get_path_data(state, reference_trajectory, N)
        cte, theta_error, index = self.compute_error(state, reference_trajectory)
        # 曲率確認
        if len(path_list)>1:
            delta_theta = self.compute_theta_error(path_list[0][2],path_list[1][2])
            omega_error=self.compute_theta_error(delta_theta, ref_delta)
        else:
            omega_error=0.0

        errors=[cte, theta_error, omega_error]
        return errors
    
    def _predict_compute(self, state, reference_trajectory, N, k=5):
        """
        起動予測制御
        """
        K_delta = {
            "theta": -0.015, "cte": 0.025, "omega": 0.02,
            "theta_trend": 0.4, "delta_theta": 0.0, "p_theta": 0.0, "p_cte": 0.0
            }
        K_accel = {
            "theta": 0.0, "cte": 0.0, "omega": -0.01,
            "theta_trend": -0.01, "path_straightness": 1.0,
            "delta_theta": 0.0, "p_theta": 0.0, "p_cte": 0.0
            }
        delta_theta_list = []
        future_err_list = {"cte": [], "theta": []}
        predict_err={"cte": 0.0, "theta": 0.0}
        omega_err=0.0
        delta2_theta=0.0
        iteration = 0
        # 現在の状態/エラー値を取得
        now_x, now_y, now_theta, now_delta, now_v, now_accel = state
        self.delta, self.accel = now_delta, now_accel
        path_list = self.get_path_data(state, reference_trajectory, N)
        now_cte, now_theta_err, now_index = self.compute_error(state, reference_trajectory)

        # 現在の制御で数ステップ分状態を確保
        pred_state_list=[]
        pred_state=self.predict_state(state, now_delta, now_accel, N=1)
        pred_state_list.append(pred_state)
        for i in range(N-1):
            pred_state=self.predict_state(pred_state, now_delta, now_accel, N=1)
            pred_state_list.append(pred_state)

        # 現在の状態で数ステップ分のエラー確認
        pred_range=min(len(path_list),N)
        for i in range(pred_range):
            future_theta_err=path_list[i][2]-pred_state_list[i][2]
            future_err_list["theta"].append(future_theta_err)
            future_cte=np.sqrt((pred_state_list[i][0] - path_list[i][0])**2 + (pred_state_list[i][1] - path_list[i][1])**2)
            future_err_list["cte"].append(future_cte)

        # 曲率確認
        for i in range(len(path_list)-1):
            delta_theta = self.compute_theta_error(path_list[i][2],path_list[i+1][2])
            delta_theta_list.append(delta_theta)
        if len(delta_theta_list)>1:
            delta2_theta=self.compute_theta_error(delta_theta_list[0],delta_theta_list[1])
            omega_err=self.compute_theta_error(delta_theta_list[1], now_delta)

        if now_v >= 0.5:
            approach=N
        else:
            approach=int(N/2)

        trend_theta_list=delta_theta_list[:min(approach,len(delta_theta_list))]
        trend_theta_list.sort(reverse=True)

        # 基本誤差による制御計算(CTE:横方向誤差、Θ：方位誤差、ω：スピン誤差(角速度誤差))
        delta = K_delta["theta"] * -now_theta_err + K_delta["cte"] * now_cte + K_delta["omega"] * omega_err
        accel = K_accel["theta"] * -now_theta_err + K_accel["cte"] * now_cte + K_accel["omega"] * omega_err

        if abs(omega_err) > 0.01:
            delta += K_delta["omega"] * abs(omega_err)
            accel += K_accel["omega"] * abs(omega_err)

        # 軌道予測制御(delta2Θ：二回微分成分、trendΘ: 予測ホライゾン上のΘ変動率/直線率)
        if len(path_list)>1:
            delta += K_delta["delta_theta"] * -delta2_theta
            accel += K_accel["delta_theta"] * -delta2_theta
            if abs(trend_theta_list[0]) > 0.02:
                delta += K_delta["theta_trend"] * -min(0.25,trend_theta_list[0])
                accel += K_accel["theta_trend"] * abs(trend_theta_list[0])
            else:
                accel += K_accel["path_straightness"] * (0.5-abs(trend_theta_list[0]))

        # 計算した制御で数ステップ分状態を確保
        if len(trend_theta_list)>0:
            trend_theta=trend_theta_list[0]
        else:
            trend_theta=0
        predict_control = [-delta, accel]
        predict_control = self.predict_horizon(state, reference_trajectory, predict_control, trend_theta, N, k)
        delta, accel = predict_control

        if len(path_list)>1:
            print(f"ステアリング: {delta:.4f}, アクセル: {accel:.4f}, 直近Θ最大変動: {trend_theta:.4f}")
        else:
            print(f"ステアリング: {delta:.4f}, アクセル: {accel:.4f}")
        print(f"目標id: {now_index:.1f}, CTE: {now_cte:.4f}, Θ誤差: {now_theta_err:.4f}, スピン誤差: {omega_err:.4f}")
        #print(f"予測精度CTE: {self.prev_predict_error["cte"]:.4f}, 予測精度Θ誤差: {self.prev_predict_error["theta"]:.4f}")
        #print(f"予測CTE変動量: {cte_trend:.4f}, 実際のCTE変動: {cte_accumulated:.4f}, CTE変動量エラー: {trend_error:.4f},速度目標: {v_goal:.4f}")

        return predict_control, pred_state

    def predict_horizon(self, state, reference_trajectory, control, theta_trend, N, k):
        delta, accel = control
        section=int(N/2)
        pred_delta_theta_list = []
        delta_theta_list = []
        future_err_list = {"cte": [], "theta": [], "omega": []}
        ref_err_list = {"cte": 0.0, "theta": 0.0, "omega": 0.0}
        pred_err_list = {"cte": 0.0, "theta": 0.0, "omega": 0.0}
        # 現在の状態/エラー値を取得
        now_x, now_y, now_theta, now_delta, now_v, now_accel = state
        path_list = self.get_path_data(state, reference_trajectory, N)
        now_errors = self.predict_compute_error(state, reference_trajectory, N)

        for m in range(k-1):
            ref_state = self.predict_state(state, -delta, accel, (m+1)*N)
            ref_errors = self.predict_compute_error(ref_state, reference_trajectory, N)
            ref_err_list["cte"] += ref_errors[0]
            ref_err_list["theta"] += ref_errors[1]
            ref_err_list["omega"] += ref_errors[2]


        pred_state = self.predict_state(state, -delta, accel, 1)
        pred_path_list = self.get_path_data(pred_state, reference_trajectory, N)

        # 未来の状態を考慮
        iteration=0
        r=0
        n_particles=10
        w, c1, c2, c3 = 0.7, 1.5, 1.5, 1.0
        delta_collection = np.linspace(-0.5, 0.5, 11)
        accel_collection = np.linspace(-0.5, 0.5, 11)

        pred_delta = np.random.choice(delta_collection, (n_particles, k))
        pred_delta *= self.steer_limit
        pred_accel = np.random.choice(accel_collection, (n_particles, k))
        pred_accel *= self.acc_limit
        pred_accel += now_v
        pred_delta_vel = np.zeros((n_particles, k))
        pred_accel_vel = np.zeros((n_particles, k))
        reward_list = []
        p_best_delta = np.zeros((n_particles, k))
        p_best_accel = np.zeros((n_particles, k))
        g_best_delta = []
        g_best_accel = []
        p_best_reward = []
        g_best_reward = 0
        g_best_index = 0
        if len(pred_path_list) > 1:
            now_range = min(N, len(pred_path_list)-1)
            target_x, target_y = pred_path_list[now_range][0], pred_path_list[now_range][1]
            while iteration<30:
                for m in range(n_particles-1):
                    for n in range(k-1):
                        pred_state = self.predict_state(state, pred_delta[m][n], pred_accel[m][n], N)
                        pred_errors = self.predict_compute_error(pred_state, reference_trajectory, N)
                        pred_err_list["cte"]+=pred_errors[0]
                        pred_err_list["theta"]+=pred_errors[1]
                        pred_err_list["omega"]+=pred_errors[2]
                    future_err_list["cte"].append(pred_err_list["cte"])
                    future_err_list["theta"].append(pred_err_list["theta"])
                    future_err_list["omega"].append(pred_err_list["omega"])
                reward_list = self.reward(future_err_list)
                for n in range(n_particles-1):
                    if iteration==0:
                        p_best_reward=reward_list
                        p_best_delta[n]=pred_delta[n]
                        p_best_accel[n]=pred_accel[n]
                    else:
                        if p_best_reward[n]>reward_list[n]:
                            p_best_reward[n]=reward_list[n]
                            p_best_delta[n]=pred_delta[n]
                            p_best_accel[n]=pred_accel[n]
                reward_copy=[]
                reward_copy=reward_list.copy()
                reward_copy.sort()
                g_best_reward=reward_copy[0]
                g_best_index=reward_list.index(g_best_reward)
                g_best_delta=pred_delta[g_best_index]
                g_best_accel=pred_accel[g_best_index]

                r1, r2 = np.random.rand(), np.random.rand()
                pred_delta_vel = w * pred_delta_vel + \
                                c1 * r1 * (p_best_delta - pred_delta) + \
                                c2 * r2 * (g_best_delta - pred_delta) + \
                                c3 * (p_best_delta - pred_delta) * (g_best_delta - pred_delta) / (abs(g_best_delta) + abs(pred_delta))
                pred_accel_vel = w * pred_accel_vel + \
                                c1 * r1 * (p_best_accel - pred_accel) + \
                                c2 * r2 * (g_best_accel - pred_accel) + \
                                c3 * (p_best_accel - pred_accel) * (g_best_accel - pred_accel) / (abs(g_best_accel) + abs(pred_accel))
                
                pred_delta += pred_delta_vel
                pred_accel += pred_accel_vel
                    
                iteration+=1

        best_delta = g_best_delta[0]
        best_accel = g_best_accel[0]

        predict_control = [best_delta, best_accel]
        return predict_control
    
    def reward(self, err_list):
        """
        err_list：cte, theta, omega
        """
        reward_list=[]
        cte_list=[]
        theta_list=[]
        omega_list=[]
        reward=0
        cte_list=err_list["cte"].copy()
        cte_list.sort()
        theta_list=err_list["theta"].copy()
        theta_list.sort()
        omega_list=err_list["omega"].copy()
        omega_list.sort()
        for k in range(len(err_list["cte"])):
            reward+=cte_list.index(err_list["cte"][k])
            # reward+=theta_list.index(err_list["theta"][k])
            reward+=omega_list.index(err_list["omega"][k])
            reward_list.append(reward)
        return reward_list
    
    def magnetic_compute(self, state, reference_trajectory, current_step, ref_state, ref_step, control, N=5, k=2):
        """
        差動二輪ロボット用：磁気双極子ベースの目的地点誘導制御
        
        Parameters
        ----------
        x, y : float
            現在位置（世界座標）
        theta : float
            現在の姿勢角 [rad]
        x_goal, y_goal : float
            目標位置
        theta_goal : float
            目標姿勢角（現在は磁場方向と一致するので未使用）
        k_omega : float
            回転ゲイン
        k_v : float
            並進速度ゲイン（cosα付き）
            
        Returns
        -------
        v : float
            並進速度
        omega : float
            角速度
        """
        k_omega = 1.0
        k_v = 1.0

        def normalize_angle(a):
            return (a + np.pi) % (2 * np.pi) - np.pi

        # ステップ1: ローカル座標系（目標位置＋姿勢を原点＋x軸とみなす）
        x, y, theta, now_delta, v, accel = state
        # path_list = self.get_path_data(x, y, reference_trajectory, N)
        # x_goal, y_goal, goal_index = self.physics.find_closest_point(x, y, reference_trajectory)
        x_goal, y_goal = reference_trajectory[ref_step]
        x_next, y_next = reference_trajectory[ref_step+1]
        theta_goal = np.arctan2(y_next - y_goal, x_next - x_goal)
        # theta_goal = path_list[1][2]
        dx = x - x_goal
        dy = y - y_goal

        cosg = np.cos(-theta_goal)
        sing = np.sin(-theta_goal)
        x_rel =  dx * cosg + dy * sing
        y_rel = -dx * sing + dy * cosg
        theta_rel = normalize_angle(theta + now_delta - theta_goal)

        # ステップ2: 磁気双極子の磁場ベクトルを算出（m = (1, 0)）
        r2 = x_rel**2 + y_rel**2 + 1e-6
        r3 = r2**1.5
        r5 = r2**2.5
        dot_mr = x_rel  # m = (1, 0)なので m・r = x_rel

        Bx = (3 * x_rel * dot_mr / r5) - (1 / r3)
        By = (3 * y_rel * dot_mr / r5)

        # Bx, By の方向だけ使い、大きさは一定にクリップ or 正規化
        norm = np.sqrt(Bx**2 + By**2) + 1e-6
        Bx /= norm
        By /= norm

        # print(f"座標: (x, y)=({x_rel}:.3f, {y_rel}:.3f), R2={r2:.3f}")

        # ステップ3: 磁場ベクトル方向に姿勢を合わせるようにωを出力
        if abs(x_rel) > 0.1 or abs(theta_rel) > 0.15:
            theta_target = np.arctan2(By, Bx)
        else:
            theta_target = 0
        alpha = normalize_angle(theta_target - theta_rel)
        omega = k_omega * alpha

        # ステップ4: 並進速度はcos(α)で前後調整（後退は抑制可）
        v = k_v * np.cos(alpha)
        v = max(0.0, v)  # 後退不可にする場合

        omega = self.theta_max * np.tanh(omega/self.theta_max/3)
        #print(f"ステアリング: {omega:.4f}, アクセル: {v:.4f}, 姿勢角: {theta+now_delta:.4f}, 軌道角度: {theta_goal:.4f}")

        predict_control = {"delta": [], "accel": []}

        predict_control["delta"].append(omega)
        predict_control["accel"].append(v)
        return predict_control, ref_state, ref_step
    
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

        now_x, now_y, now_theta, now_delta, now_v, now_accel = state

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
            diff = []
            for i in range(1, len(log)-1):
                diff.append((log[i+1]-log[i]) if log[i+1] > 0 else (log[i]-log[i+1]))
            return diff
        
        def calculate_mean(log, gamma=0.9):
            mean = 0.0
            weight = gamma ** np.arange(len(log))
            weigts = np.sum(weight)
            for i in range(1, len(log)-1):
                mean += abs(log[i])
            mean /= weigts
            return mean
        
        # --- 仮走行を1粒子ぶんシミュレーション ---
        def simulate_particle(state, kq):
            q12, q13, q14, q15, q23, q24, q25, q34, q35, q45 = kq
            pos_err, theta_err, cte, omega_err, now_index = self.compute_error(state, reference_trajectory, now_x, now_y)
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
                pred_state=self.predict_state(base_state, delta, now_accel, N=1)
                pred_x, pred_y, pred_theta, pred_delta, pred_v, pred_accel = pred_state
                pos_err, theta_err, cte, omega_err, now_index = self.compute_error(pred_state, reference_trajectory, pred_x, pred_y)
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
        pos_err, theta_err, cte, omega_err, now_index = self.compute_error(state, reference_trajectory, now_x, now_y)
        omega = base_omega(cte, omega_err, theta_err, now_v, best_p)
        omega = self.theta_max * np.tanh(omega/self.theta_max/3)
        accel = k_v * np.cos(omega)
        print(f"最適化終了: {best_cost:.3f} / delta : {omega:.3f}")
        predict_control = {"delta": [], "accel": []}

        predict_control["delta"].append(omega)
        predict_control["accel"].append(accel)

        return predict_control, ref_state, ref_step