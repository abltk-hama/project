# Re-run the Dubins implementation example after reset
from config import Config
import numpy as np
import math
import matplotlib.pyplot as plt

# --- Dubins Path Utility Functions ---
class DubinsPath:
    def __init__(self):
        config = Config()
        params = config.get_vehicle_params()
        self.v_max, self.v_min = 0.28, 0.07
        self.deg = 18
        self.theta_max, self.theta_min = self.deg/180*np.pi, -self.deg/180*np.pi
        # 車両設定を読み込み
        self.wheel_base = params["wheel_base"] 
        self.wheel_radius = params["wheel_radius"] 

        self.rho=self.wheel_base/np.tan(self.theta_max)
        self.step_size=0.5

    def mod2pi(self, theta):
        return theta - 2.0 * math.pi * math.floor(theta / (2.0 * math.pi))

    def dubins_LSL(self, alpha, beta, d):
        sa, sb = math.sin(alpha), math.sin(beta)
        ca, cb = math.cos(alpha), math.cos(beta)
        tmp = 2 + d*d - 2*ca*cb + 2*d*(sa - sb)
        if tmp < 0:
            return None
        p = math.sqrt(tmp)
        tmp2 = math.atan2((cb - ca), (d + sa - sb))
        t = self.mod2pi(-alpha + tmp2)
        q = self.mod2pi(beta - tmp2)
        return (t, p, q)

    def dubins_RSR(self, alpha, beta, d):
        sa, sb = math.sin(alpha), math.sin(beta)
        ca, cb = math.cos(alpha), math.cos(beta)
        tmp = 2 + d*d - 2*ca*cb + 2*d*(sb - sa)
        if tmp < 0:
            return None
        p = math.sqrt(tmp)
        tmp2 = math.atan2((ca - cb), (d - sa + sb))
        t = self.mod2pi(alpha - tmp2)
        q = self.mod2pi(-beta + tmp2)
        return (t, p, q)

    def dubins_LSR(self, alpha, beta, d):
        sa, sb = math.sin(alpha), math.sin(beta)
        ca, cb = math.cos(alpha), math.cos(beta)
        tmp = -2 + d*d + 2*ca*cb + 2*d*(sa + sb)
        if tmp < 0:
            return None
        p = math.sqrt(tmp)
        tmp2 = math.atan2((-ca - cb), (d + sa + sb)) - math.atan2(-2.0, p)
        t = self.mod2pi(-alpha + tmp2)
        q = self.mod2pi(-self.mod2pi(beta) + tmp2)
        return (t, p, q)

    def dubins_RSL(self, alpha, beta, d):
        sa, sb = math.sin(alpha), math.sin(beta)
        ca, cb = math.cos(alpha), math.cos(beta)
        tmp = d*d - 2 + 2*ca*cb - 2*d*(sa + sb)
        if tmp < 0:
            return None
        p = math.sqrt(tmp)
        tmp2 = math.atan2((ca + cb), (d - sa - sb)) - math.atan2(2.0, p)
        t = self.mod2pi(alpha - tmp2)
        q = self.mod2pi(beta - tmp2)
        return (t, p, q)

    def dubins_RLR(self, alpha, beta, d):
        sa, sb = math.sin(alpha), math.sin(beta)
        ca, cb = math.cos(alpha), math.cos(beta)
        tmp = (6 - d*d + 2*ca*cb + 2*d*(sa - sb)) / 8
        if abs(tmp) > 1:
            return None
        p = self.mod2pi(2*math.pi - math.acos(tmp))
        t = self.mod2pi(alpha - math.atan2(ca - cb, d - sa + sb) + p/2)
        q = self.mod2pi(alpha - beta - t + p)
        return (t, p, q)

    def dubins_LRL(self, alpha, beta, d):
        sa, sb = math.sin(alpha), math.sin(beta)
        ca, cb = math.cos(alpha), math.cos(beta)
        tmp = (6 - d*d + 2*ca*cb + 2*d*(sb - sa)) / 8
        if abs(tmp) > 1:
            return None
        p = self.mod2pi(2*math.pi - math.acos(tmp))
        t = self.mod2pi(-alpha - math.atan2(ca - cb, d + sa - sb) + p/2)
        q = self.mod2pi(beta - alpha - t + p)
        return (t, p, q)
    
        # --- Main Function ---
    def dubins_shortest_path(self, start, goal, rho=1.0, step_size=0.05):
        dx, dy = (goal[0]-start[0])/rho, (goal[1]-start[1])/rho
        D = math.hypot(dx, dy)
        d = D
        theta = self.mod2pi(math.atan2(dy, dx))
        alpha = self.mod2pi(start[2] - theta)
        beta = self.mod2pi(goal[2] - theta)

        path_types = {
            "LSL": self.dubins_LSL,
            "RSR": self.dubins_RSR,
            "LSR": self.dubins_LSR,
            "RSL": self.dubins_RSL,
            "RLR": self.dubins_RLR,
            "LRL": self.dubins_LRL,
        }

        best_cost, best_path, best_type = float("inf"), None, None
        for key, func in path_types.items():
            val = func(alpha, beta, d)
            if val:
                cost = sum(val)
                if cost < best_cost:
                    best_cost, best_path, best_type = cost, val, key

        # サンプリング
        x, y, th = start
        points = [(x, y)]
        t, p, q = best_path
        segs = [t, p, q]
        dirs = list(best_type)
        first_dir = dirs[0]
        for seg, direc in zip(segs, dirs):
            length = seg * rho
            step = step_size * rho
            n = int(abs(length/step))
            for _ in range(n):
                if direc == "L":
                    th += step/rho
                elif direc == "R":
                    th -= step/rho
                x += step*math.cos(th)
                y += step*math.sin(th)
                points.append((x, y))
        return points, best_type, first_dir

    def dubins_compute(self, state, reference_trajectory, current_step, ref_state, ref_step, control, N=5, k=2):
        x, y, theta, now_delta, v, accel = state
        start = (x, y, theta)
        x_tg, y_tg = reference_trajectory[ref_step]
        x_next, y_next = reference_trajectory[ref_step + 1]
        theta_tg = math.atan2(y_next - y_tg, x_next - x_tg)
        goal = (x_tg, y_tg, theta_tg)
        distance = np.sqrt((x - x_tg) ** 2 + (y - y_tg) ** 2)/2
        pts, path_type, dict = self.dubins_shortest_path(start, goal, self.rho, self.step_size)
        steering_angles = np.atan(self.wheel_base/self.rho) if dict == "L" else -np.atan(self.wheel_base/self.rho) if dict == "R" else 0.0
        steering_angles = self.theta_max * np.tanh(steering_angles/self.theta_max/3)
        accel = self.v_max * np.tanh(distance/self.v_max/3)
        predict_control = {"delta": [], "accel": []}

        predict_control["delta"].append(steering_angles)
        predict_control["accel"].append(accel)

        return predict_control, ref_state, ref_step
