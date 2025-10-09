# [MODULE]: config
[PURPOSE]:
  シミュレーション・実機制御に共通する定数・設定を管理。

[RESPONSIBILITY]:
  - 物理パラメータ（ホイールベース、トレッド幅など）
  - 制御パラメータ（ゲイン、制約角度など）
  - 実験条件（初期位置、タイムステップなど）

[DEPENDS_ON]:
  - （なし）

[USED_BY]:
  - controllers.*
  - models.physics
  - sim.simulation

---

## 📂 ファイル一覧

| ファイル名 | 概要 | 主な定義 |
|:--|:--|:--|
| `constants.py` | 物理パラメータを定義。 | `WHEEL_BASE`, `TRACK_WIDTH`, `DT` |
| `controller_params.py` | 制御ゲインや閾値を定義。 | `K_CTE`, `K_THETA`, `DELTA_MAX` |
| `paths.py` | 使用する経路ファイルや設定。 | `DEFAULT_PATH`, `LOOKAHEAD_DIST` |

---

## 🧠 AI補助向けメモ
- AIはこれらを**「依存ではなく参照」**として扱う。  
- 変更の影響範囲が広いため、提案時は参照箇所を明示する必要あり。