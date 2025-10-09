# [MODULE]: controllers
[PURPOSE]:
  経路追従・姿勢制御などの上位制御アルゴリズムをまとめる層。

[SUBMODULES]:
  - path_follow: 経路追従制御（CTE, θerr, ωerrに基づく）
  - pose_regulation: 目的地点モード。Lyapunov関数制御やDubins経路を採用。
  - pso_adapt: 適応的パラメータ最適化制御。

[INTERFACE]:
  入力: 現在位置、姿勢角、経路情報  
  出力: v（並進速度）, δ（操舵角）

[DEPENDS_ON]:
  - models.physics
  - trajectory.utils
  - utils.math_tools

[USED_BY]:
  - sim.simulation