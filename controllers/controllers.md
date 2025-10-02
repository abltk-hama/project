# controllers ディレクトリ

制御アルゴリズムをまとめる。経路追従、Dubins、PSO適応制御、強化学習制御など複数の手法を共存させる。

---

## trajectory_control.py
経路追従制御と目的地点モードの切り替えを扱う。

### 関数一覧
- `trajectory_following_control` : 経路上の姿勢差・CTEを用いて追従。
- `goal_point_control` : DubinsやPose Regulationによる目的地点到達。

---

## simple_predict.py（→ predict_control.py）
予測制御の基盤。物理モデルと誤差計算を組み合わせて評価。

### 関数一覧
- `predict_horizon` : 複数ステップ先の誤差を評価。
- `evaluate_control` : 誤差と滑らかさを元にスコア化。

---

## pso_adaptive.py
PSOを用いた係数最適化型の適応制御。

---

## rl_control.py
強化学習制御用。DQNやActor-Criticで制御ポリシーを学習する。
