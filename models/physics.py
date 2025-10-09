import numpy as np
from config.config import Config
from models.interfaces import VehicleInterface

class Physics:
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
        self.delta = 0.0
        self.accel = 0.0
        self.last_vL, self.last_vR = 0.0, 0.0

    def initial_state(self, x, y, theta, omega, v):
        self.x = x
        self.y = y
        self.omega = omega
        self.theta = theta
        self.v = v

    def step(self, delta, v):
        """ステアリング角 delta [rad]、速度 v [m/s] から車両の挙動を更新"""
        self.delta = delta
        if abs(delta) > 1E-4:  # ステアリングがほぼゼロでなければ
            R = self.wheel_base / np.tan(delta)  # 曲率半径
            omega = self.v / R * self.dt  # 角速度
        else:
            omega = 0
            L = self.v * self.dt  # 角速度

        self.last_vL = v - (omega * self.wheel_base) / 2
        self.last_vR = v + (omega * self.wheel_base) / 2

        raw_vL = (self.last_vL - self.v) * (1 - np.exp(-1*self.dt))
        raw_vR = (self.last_vR - self.v) * (1 - np.exp(-1*self.dt))

        self.omega = (raw_vR - raw_vL) / self.wheel_base
        self.v = (raw_vL + raw_vR) / 2

        if abs(self.omega) < 1E-4:  # ステアリングがほぼゼロなら直進
            L = self.v * self.dt
        else:
            L = 2 * R * np.sin(self.omega / 2)  # 弧の長さ

        next_theta = self.theta + self.omega / 2
        self.x += L * np.cos(next_theta)
        self.y += L * np.sin(next_theta)
        self.theta += self.omega

        return self.x, self.y, self.theta, self.omega, self.v
    
    def get_state(self):
        """ 現在の車両状態を取得 """
        return self.x, self.y, self.theta, self.omega, self.v