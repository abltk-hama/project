# [MODULE]: trajectory
[PURPOSE]:
  経路生成・補間・姿勢情報の整形など、経路データの処理を担当。

[RESPONSIBILITY]:
  - 経路点列の生成と補間
  - 経路上の姿勢角・曲率・接線方向の計算
  - 経路上の最近点探索および lookahead 点算出

[DEPENDS_ON]:
  - models.geometry
  - utils.math_tools

[USED_BY]:
  - controllers.path_follow
  - sim.simulation

---

## 📂 ファイル一覧

| ファイル名 | 概要 | 主な関数・クラス |
|:--|:--|:--|
| `planner.py` | 経路生成および補間を担当。直線・円弧・S字などの経路を出力。 | `generate_path()`, `interpolate_curve()` |
| `utils.py` | 経路点の補助演算。最近点や接線方向の算出を提供。 | `find_closest_point()`, `compute_tangent_angle()`, `lookahead_point()` |

---

## 🧩 関数リスト（全体要約）

| 関数 / クラス名 | 入出力 | 概要 | 所属ファイル |
|:--|:--|:--|:--|
| `generate_path()` | (path_type, params) → list | 経路を生成し、座標点列を返す | planner.py |
| `find_closest_point()` | (pos, path) → tuple | 車両位置から最も近い経路上の点を返す | utils.py |
| `compute_tangent_angle()` | (p1, p2) → float | 経路の局所的な接線角を求める | utils.py |

---

## 🧠 AI補助向けメモ
- この層は「経路情報の生成・管理」であり、制御には関与しない。  
- 経路追従で利用されるのは「最近点」「接線方向」「局所曲率」。  
- AIに依頼する場合：
  - **経路生成アルゴリズム改善** → `planner.py`  
  - **補助関数の最適化** → `utils.py`