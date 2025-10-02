# models ディレクトリ
車両モデルや物理モデルをまとめる。

---

## physics.py
物理計算を行うための関数をまとめる。

### 関数一覧
- `compute_distance_error / compute_errors` : 距離誤差計算
- `compute_angle_error` : 角度誤差計算
- `compute_curvature` : 曲率計算

## real_interface.py
実機で制御する際の物理特性や風、路面からの影響などを考慮するためのロバスト関連。

## drive.py
差動二輪の運動モデル。ハンドルや左右タイヤの速度を与えて次の状態を求める。
