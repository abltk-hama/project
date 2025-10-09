"""
VehicleInterface
---------------------------------------------------
シミュレーション（Physics）および実機制御（RealVehicle）の
共通インターフェース定義。

simulation.py からはこのインターフェースを通して呼び出す。
"""

from abc import ABC, abstractmethod
from typing import Dict

class VehicleInterface(ABC):
    """差動二輪ロボット共通インターフェース"""

    @abstractmethod
    def update_state(self, delta: float, v: float) -> None:
        """ステア角δ[rad]と速度v[m/s]を入力して内部状態を更新"""
        pass

    @abstractmethod
    def get_state(self) -> Dict[str, float]:
        """現在の車両状態(x, y, θ, v, ωなど)を辞書で返す"""
        pass

    @abstractmethod
    def stop(self) -> None:
        """車両の停止処理"""
        pass

    @abstractmethod
    def reset(self) -> None:
        """初期化処理"""
        pass