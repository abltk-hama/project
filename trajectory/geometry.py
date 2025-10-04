import numpy as np

class geometry:
    def __init__(self):
        pass

    def normalize_angle(self, angle):
        """角度を -π から π の範囲に正規化"""
        return (angle + np.pi) % (2 * np.pi) - np.pi
    
    def normalize_delta_angle(self, minuend, subtrahend):
        """2つの角度の差を -π から π の範囲に正規化"""
        delta_angle = minuend - subtrahend
        return (delta_angle + np.pi) % (2 * np.pi) - np.pi

    def compute_curvature(self, trajectory, idx):
        """
        軌道 `trajectory` の `idx` 番目の点での曲率を計算する
        """
        trajectory = np.array(trajectory)  # ✅ NumPy 配列に変換

        # ✅ インデックスの範囲外を防ぐ（境界条件）
        if idx <= 0 or idx >= len(trajectory) - 1:
            return 0.0, None  # 端では曲率が定義できない

        # ✅ 前後の3点を取得
        x1, y1 = trajectory[idx - 1]
        x2, y2 = trajectory[idx]
        x3, y3 = trajectory[idx + 1]

        # ✅ 3点の円の中心を求める（外接円の中心）
        A = np.array([
            [2 * (x2 - x1), 2 * (y2 - y1)],
            [2 * (x3 - x2), 2 * (y3 - y2)]
        ])
        b = np.array([
            (x2**2 - x1**2) + (y2**2 - y1**2),
            (x3**2 - x2**2) + (y3**2 - y2**2)
        ])

        try:
            cx, cy = np.linalg.solve(A, b)  # ✅ 連立方程式を解く
            R = np.linalg.norm([x2 - cx, y2 - cy])  # ✅ 半径を計算
            curvature = 1 / R if R != 0 else 0  # ✅ 曲率 κ = 1/R
        except np.linalg.LinAlgError:
            curvature = 0  # ✅ 3点が一直線の場合、曲率はゼロ

        # ✅ 軌道の方向（目標角度）を計算
        theta_target = np.arctan2(y3 - y1, x3 - x1)  # ✅ 目標方向の角度

        return curvature, theta_target
    
    def get_path_data(self, state, reference_trajectory, N):
        """経路情報を取得する"""
        x, y, theta, _, _ = state
        # 現在の速度を元に取得する経路情報を計算
        section=N*2
        # 現在位置から最も近いポイントを探す
        x_ref, y_ref, ref_index = self.find_closest_point(x, y, reference_trajectory)

        # 経路情報を取得[x, y, theta]
        distance = 0
        path_list=[]
        prev_x, prev_y = x_ref, y_ref
        for i in range(ref_index+1, len(reference_trajectory)):
            x, y = reference_trajectory[i]
            distance += np.sqrt((x - prev_x) ** 2 + (y - prev_y) ** 2)
            prev_path_theta = np.arctan2(y - prev_y, x - prev_x)
            prev_path=[prev_x, prev_y, prev_path_theta]
            path_list.append(prev_path)
            prev_x, prev_y = x, y
            if distance > section:
                break
        return path_list
    
    def path_predict(self, path, v, N):
        # 現在の状態で数ステップ分のエラー確認
        delta_theta_list=[]
        trend_theta=0.0
        delta2_theta=0.0

        # 曲率確認
        for i in range(len(path)-1):
            delta_theta = path[i][2] - path[i+1][2]
            delta_theta_list.append(delta_theta)
        if len(delta_theta_list)>1:
            delta2_theta=self.compute_theta_error(delta_theta_list[0],delta_theta_list[1])
            approach=N if v>=0.5 else int(N/2)
            trend_theta_list=delta_theta_list[:min(approach,len(delta_theta_list))]
            trend_theta_list.sort(reverse=True)
            trend_theta=trend_theta_list[0]

        return trend_theta, delta2_theta
    
    def find_target_point(self, state, trajectory, current_step, lookahead_distance):
        """現在の位置から lookahead_distance 先の目標点を探す"""
        x_cur, y_cur, theta, _, _ = state
        
        for i in range(current_step, len(trajectory)):
            x, y = trajectory[i]
            distance = np.sqrt((x - x_cur) ** 2 + (y - y_cur) ** 2)

            if distance >= lookahead_distance:
                return x, y, i  # ✅ 指定距離に達したらその点を返す
        
        return x, y, len(trajectory)-2  # ✅ 終端に達したら最後の点を返す
    
    def find_closest_point(self, x, y, trajectory):
        """
        軌道 `trajectory` 上の最も近い点を探す
        """
        trajectory = np.array(trajectory)  # ✅ NumPy 配列に変換
        distances = np.linalg.norm(trajectory - np.array([x, y]), axis=1)  # ✅ 全点との距離を計算
        idx_closest = np.argmin(distances)  # ✅ 最も近い点のインデックスを取得
        
        x_closest, y_closest = trajectory[idx_closest]  # ✅ 最も近い点の座標を取得
        return x_closest, y_closest, idx_closest
    
    def find_forward_point(self, x, y, trajectory):
        """
        軌道 `trajectory` 上で前方の最も近い点を探す
        """
        trajectory = np.array(trajectory)  # ✅ NumPy 配列に変換
        distances = np.linalg.norm(trajectory - np.array([x, y]), axis=1)  # ✅ 全点との距離を計算
        idx_closest = np.argmin(distances)  # ✅ 最も近い点のインデックスを取得
        x_rel = 0.1

        while x_rel > 0.0 and idx_closest <= len(trajectory)-2:  # 前方の点を探す
            x_goal, y_goal = trajectory[idx_closest]
            x_next, y_next = trajectory[idx_closest+1]
            theta_goal = np.arctan2(y_next - y_goal, x_next - x_goal)
            # theta_goal = path_list[1][2]
            dx = x - x_goal
            dy = y - y_goal
            cosg = np.cos(-theta_goal)
            sing = np.sin(-theta_goal)
            x_rel =  dx * cosg + dy * sing
            y_rel = -dx * sing + dy * cosg
            if x_rel >= 0.0:
                idx_closest += 1
        
        x_closest, y_closest = trajectory[idx_closest]  # ✅ 最も近い点の座標を取得
        return x_closest, y_closest, idx_closest

    def compute_cte(self, state, trajectory):
        """
        Compute the Cross-Track Error (CTE) between the current position and the trajectory.

        Parameters:
        position (tuple): Current position as (x, y(, theta, omega, v)).
        trajectory (list): List of waypoints [(x1, y1), (x2, y2), ...].

        Returns:
        float: The computed CTE.
        """
        x, y, theta, _, _ = state
        x_ref, y_ref, ref_index = self.find_forward_point(x, y, trajectory)

        CTE = np.linalg.norm([x - x_ref, y - y_ref])  # 横方向誤差
        return CTE

    def compute_heading_error(self, state, trajectory, idx):
        """
        Compute the heading error between the vehicle's current heading and the trajectory's heading at index idx.

        Parameters:
        state (tuple): Current state as (x, y, theta, v, delta, a).
        trajectory (list): List of waypoints [(x1, y1), (x2, y2), ...].
        idx (int): Index of the trajectory point to compare against.

        Returns:
        float: The computed heading error in radians.
        """
        x, y, theta, _, _ = state

        if idx < 0 or idx >= len(trajectory) - 1:
            return 0.0  # Invalid index

        x1, y1 = trajectory[idx]
        x2, y2 = trajectory[idx + 1]

        path_theta = np.arctan2(y2 - y1, x2 - x1)

        heading_error = path_theta - theta
        # Normalize to [-pi, pi]
        heading_error = (heading_error + np.pi) % (2 * np.pi) - np.pi

        return heading_error
    
    def compute_omega_error(self, state, path):
        x, y, theta, delta, v = state
        # 曲率確認
        if len(path)>1:
            delta_theta = path[0][2] - path[1][2]
            delta_theta = (delta_theta + np.pi) % (2 * np.pi) - np.pi
            omega_error = delta_theta - delta
            omega_error = (omega_error + np.pi) % (2 * np.pi) - np.pi
        else:
            omega_error=0.0
        return omega_error