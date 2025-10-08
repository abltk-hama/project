"""
RealVehicle: 実機モータ制御クラス
--------------------------------
- Raspberry Pi の I2C 経由でモータドライバを制御。
- シミュレーション用 VehicleInterface と共通のメソッド構造を持つ。
"""

import time
import math
import smbus
from pathlib import Path

# param_ids.py を読み込み（project階層からの相対パス）
from i2c_controller.params.param_ids import PARAMS

class RealVehicle:
    """実機制御用の差動二輪ロボット制御クラス"""

    def __init__(self, i2c_bus_id=1, i2c_addr=0x08, wheel_base=0.5, wheel_radius=0.05):
        """
        Parameters
        ----------
        i2c_bus_id : int
            I2Cバス番号（通常は1）
        i2c_addr : int
            モータ制御ボードのアドレス
        wheel_base : float
            車輪間距離 [m]
        wheel_radius : float
            車輪半径 [m]
        """
        self.bus = smbus.SMBus(i2c_bus_id)
        self.addr = i2c_addr
        self.wheel_base = wheel_base
        self.wheel_radius = wheel_radius

        # 実機状態（簡易的に保持）
        self.v = 0.0       # 並進速度
        self.delta = 0.0   # ステア角
        self.encoder_L = 0.0
        self.encoder_R = 0.0

    # ================================================================
    # --- 基本通信 ---
    # ================================================================
    def write_param(self, param_id, value):
        """I2C書き込み"""
        try:
            # valueを整数に変換して送信
            self.bus.write_word_data(self.addr, param_id, int(value))
        except Exception as e:
            print(f"[I2C Error] write_param({param_id}, {value}): {e}")

    def read_param(self, param_id):
        """I2C読み出し"""
        try:
            val = self.bus.read_word_data(self.addr, param_id)
            return val
        except Exception as e:
            print(f"[I2C Error] read_param({param_id}): {e}")
            return None

    # ================================================================
    # --- 差動二輪モデルに基づく出力計算 ---
    # ================================================================
    def update_motor_output(self, v, delta):
        """
        車両速度 v [m/s] とステア角 delta [rad] から
        左右モータの速度指令を生成して送信。
        """
        self.v = v
        self.delta = delta

        # 差動二輪モデルに基づく左右速度計算
        if abs(delta) < 1e-6:
            vL = vR = v
        else:
            R = self.wheel_base / math.tan(delta)
            vL = v * (R - self.wheel_base / 2) / R
            vR = v * (R + self.wheel_base / 2) / R

        # PWM単位変換 (例: m/s → 0-255)
        pwm_L = self._convert_velocity_to_pwm(vL)
        pwm_R = self._convert_velocity_to_pwm(vR)

        # I2C経由で送信
        self.write_param(PARAMS["VEL_LEFT"]["id"], pwm_L)
        self.write_param(PARAMS["VEL_RIGHT"]["id"], pwm_R)

        # ローカル状態更新
        self.encoder_L += vL * 0.1
        self.encoder_R += vR * 0.1

    def _convert_velocity_to_pwm(self, v_mps, max_v=1.0):
        """m/sを0〜255のPWM値に変換"""
        pwm = int(max(min(v_mps / max_v * 255, 255), -255))
        return pwm

    # ================================================================
    # --- 高レベルインターフェース ---
    # ================================================================
    def set_control(self, v, delta):
        """統一インターフェース：速度・操舵角を設定"""
        self.update_motor_output(v, delta)

    def get_state(self):
        """現在の推定状態を返す（実機では簡易的にエンコーダベース）"""
        return {
            "v": self.v,
            "delta": self.delta,
            "encoder_L": self.encoder_L,
            "encoder_R": self.encoder_R
        }

    def stop(self):
        """停止コマンド"""
        self.write_param(PARAMS["VEL_LEFT"]["id"], 0)
        self.write_param(PARAMS["VEL_RIGHT"]["id"], 0)
        self.v = 0.0

    # ================================================================
    # --- 終了処理 ---
    # ================================================================
    def disconnect(self):
        """I2Cバスを安全にクローズ"""
        try:
            self.stop()
            self.bus.close()
            print("[RealVehicle] I2C connection closed.")
        except Exception as e:
            print(f"[RealVehicle] Disconnect error: {e}")