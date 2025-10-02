import numpy as np

class Physics:
    """
    @staticmethod
    def find_target_point(state, trajectory, lookahead_distance):
        ""Pure Pursuit: 目標点を探す""
        x, y, theta = state
        x_path, y_path = np.array(trajectory).T

        distances = np.sqrt((x_path - x)**2 + (y_path - y)**2)
        target_idx = np.argmin(np.abs(distances - lookahead_distance))
        return x_path[target_idx], y_path[target_idx]
    """

    @staticmethod
    def calculate_angle_error(state, target):
        """目標点に向かうための角度誤差を計算"""
        x, y, theta = state
        x_target, y_target = target
        angle_to_target = np.arctan2(y_target - y, x_target - x)
        return angle_to_target - theta

    @staticmethod
    def compute_distance_error(state, target):
        """距離誤差を計算"""
        x, y, _ = state
        x_target, y_target = target
        return np.sqrt((x_target - x) ** 2 + (y_target - y) ** 2)
    
    def compute_errors(self, state, target):
        """
        車両の現在状態と目標状態の誤差を計算する。
        
        Parameters:
        - state: (x, y, theta) 現在の車両の状態
        - target: (x_target, y_target) 目標点の座標
        
        Returns:
        - distance_error: 目標点までの距離誤差
        - angle_error: 目標点に対する角度誤差
        """
        x, y, theta = state
        x_target, y_target = target
        
        # ✅ ユークリッド距離誤差を計算
        distance_error = np.linalg.norm([x_target - x, y_target - y])

        # ✅ 目標点への角度を計算
        angle_to_target = np.arctan2(y_target - y, x_target - x)

        # ✅ 角度誤差を計算（現在の向きと目標角度の差）
        angle_error = angle_to_target - theta

        # ✅ 角度誤差を -π 〜 π の範囲に正規化
        angle_error = (angle_error + np.pi) % (2 * np.pi) - np.pi

        return distance_error, angle_error
    
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
    
    def find_target_point(self, state, trajectory, current_step, lookahead_distance):
        """現在の位置から lookahead_distance 先の目標点を探す"""
        x_current, y_current, theta = state
        
        for i in range(current_step, len(trajectory)):
            x, y = trajectory[i]
            distance = np.sqrt((x - x_current) ** 2 + (y - y_current) ** 2)

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
        軌道 `trajectory` 上の最も近い点を探す
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
