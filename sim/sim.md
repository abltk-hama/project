# [MODULE]: sim
[PURPOSE]:
  制御・物理モデル・経路情報を統合し、走行シミュレーションを実行する。

[RESPONSIBILITY]:
  - 各制御モード（経路追従／目的地点）の切替
  - 車両状態更新ループの管理
  - 結果の記録・描画

[DEPENDS_ON]:
  - controllers.*
  - models.*
  - trajectory.*

[USED_BY]:
  - 実行スクリプト（`main.py` など）

---

## 📂 ファイル一覧

| ファイル名 | 概要 | 主な関数・クラス |
|:--|:--|:--|
| `simulation.py` | メインの制御ループ。状態更新と制御切替を管理。 | `simulate()`, `run_step()`, `log_results()` |
| `vehicle_manager.py` | 実機／仮想車両の切替。モデルとインターフェースを抽象化。 | `VehicleManager`, `switch_mode()` |

---

## 🧩 関数リスト（全体要約）

| 関数 / クラス名 | 入出力 | 概要 | 所属ファイル |
|:--|:--|:--|:--|
| `simulate()` | (path, mode) → log | 経路を入力として走行シミュレーションを実行 | simulation.py |
| `run_step()` | (state, control) → state | 1ステップ分の状態更新 | simulation.py |
| `VehicleManager` | class | 実機 or 仮想モデルの切替を管理 | vehicle_manager.py |

---

## 🧠 AI補助向けメモ
- AIが扱う際は、ここを「全体統合層」として扱う。  
- **制御**を議論するときは controllers へ、**物理的挙動**は models へ誘導。  
- **テストパラメータ最適化**時は sim 側に entry point を設けるのが安全。