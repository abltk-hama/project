# trajectory_schema.md

## PURPOSE  
経路データ(JSON)の標準構造を定義し、  
Excelテンプレート(`データ構造.xlsm`)との対応を明示する。  
本仕様に従うことで、経路追従・目的地点制御に共通のフォーマットを用いることができる。

---

## JSON STRUCTURE OVERVIEW

```json
{
  "trajectory_id": "path_001",
  "description": "Example trajectory combining line and arc segments",
  "elements": [
    {
      "id": "seg_01",
      "type": "line",
      "start": [0.0, 0.0, 0.0],
      "end": [2.0, 0.0, 0.0],
      "speed": 0.2
    },
    {
      "id": "seg_02",
      "type": "arc",
      "center": [2.0, 1.0],
      "radius": 1.0,
      "start_angle": 270,
      "end_angle": 180,
      "direction": "CCW",
      "speed": 0.2
    }
  ]
}
```

---

## FIELD DEFINITIONS

| Key | Type | Description | Example | Excel列 |
|------|------|-------------|----------|-----------|
| trajectory_id | str | 経路全体の識別子 | "path_001" | TrajectoryID |
| description | str | 経路の説明（任意） | "Line + Arc test path" | Description |
| elements | list | 区間の配列 | - | - |
| id | str | 区間の識別子 | "seg_01" | SegmentID |
| type | str | 区間の種類 ("line" or "arc") | "line" | Type |
| start | [float, float, float] | 始点の座標と姿勢角(°) | [0.0, 0.0, 0.0] | StartX, StartY, Startθ |
| end | [float, float, float] | 終点の座標と姿勢角(°) | [2.0, 0.0, 0.0] | EndX, EndY, Endθ |
| center | [float, float] | 円弧の中心座標 | [2.0, 1.0] | CenterX, CenterY |
| radius | float | 円弧の半径(m) | 1.0 | Radius |
| start_angle | float | 円弧開始角度(°) | 270 | StartAngle |
| end_angle | float | 円弧終了角度(°) | 180 | EndAngle |
| direction | str | 回転方向("CW" or "CCW") | "CCW" | Direction |
| speed | float | 推奨速度[m/s] | 0.2 | Speed |

---

## CORRESPONDENCE WITH EXCEL TEMPLATE (`データ構造.xlsm`)

| Excelシート | 用途 | 備考 |
|--------------|------|------|
| **TrajectoryList** | 経路IDと概要の一覧 | trajectory_id, description を定義 |
| **Elements** | 各区間(line/arc)の詳細設定 | 上記フィールドに対応 |
| **Config** | JSON出力/読込設定, 分割数など | trajectory_loader用パラメータ |

> Excel側の各行が JSON 内の `"elements"` 配列の 1 要素に対応。  
> VBAマクロでシート→JSON変換、またはPythonで `openpyxl` を使った双方向変換を予定。

---

## UTILIZATION IN SYSTEM

| 用途 | ファイル | 処理概要 |
|------|----------|----------|
| 経路データ読込 | `trajectory_loader.py` | JSONをパースし、点列を生成 |
| 経路点生成 | `geometry.py` | line: 線形補間, arc: パラメトリック補間 |
| 経路追従制御 | `controllers/path_follow.py` | CTE・θerr計算に使用 |
| 目的地点制御 | `controllers/pose_regulation.py` | 終点データを目標点として利用 |

---

## FUTURE EXTENSIONS
- 複合経路(`"type": "complex"`)の追加（複数arc/line混合）
- `behavior_hint`: 経路特性を制御アルゴリズムに渡す（例：sharp_turn, slow_zone）
- `meta`: 出典・作成者・日付などのメタ情報追加
- Excel⇄JSON同期を行う `trajectory_exporter.py` / `trajectory_importer.py` の整備

---

📘 **補足**  
AIはこのスキーマを参照して以下を判断できます：
- JSONの各キーがどの処理階層で利用されるか  
- Excel列と直接対応しているため、人間が修正した場合でも追跡可能  
- geometry/physics層に与えるデータの意味を自動的に解釈できる
