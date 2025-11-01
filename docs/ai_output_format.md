# ai_output_format.md
\---

## [PURPOSE]
本ファイルは、AIが生成する出力物（Markdown, Python, JSON など）の  
**形式構造・章立て・整形ルール（Whatを出すか）** を定義する。  
AIの出力行動そのものは [ai_output_rules.md](./ai_output_rules.md) に準拠する。

---

## [1. STANDARD MARKDOWN STRUCTURE]

すべてのmdドキュメントは以下の構成を推奨する：

```
# ファイル名
---
## [PURPOSE]
## [FILES & RESPONSIBILITIES]
## [STRUCTURE SUMMARY]
## [UPDATE LOG]
```

> 各セクションはAIと人間の両方が解釈できる構造を維持する。

---

## [2. HEADER & SECTION FORMAT]

| セクション | 用途 |
|-------------|------|
| `[PURPOSE]` | ファイルや階層の目的・設計意図を記載。 |
| `[FILES & RESPONSIBILITIES]` | ファイル単位の責務・主要関数を記載。 |
| `[STRUCTURE SUMMARY]` | ディレクトリ構成・依存関係・Mermaid図を記載。 |
| `[UPDATE LOG]` | 日時（M/D/N/B形式）と修正概要を記載。 |

---

## [3. CODE OUTPUT FORMAT]

1. コードブロックは明示的に言語を指定する（例：```python```）。  
2. ファイル名は最初にコメントで示す（例：`# controllers/path_follow.py`）。  
3. 複数ファイルをまとめる場合は `\--- ファイル名 ---` で区切る。  
4. 出力例やJSON構造には必ずインデントを揃える。  

---

## [4. DATE FORMAT SPEC]

すべての更新ログ・生成物は以下の時間帯コード方式に基づき記録される。  
（詳細ルールは [ai_output_rules.md](./ai_output_rules.md) を参照）

| コード | 時間帯 | 意味 |
|:--:|:--:|:--|
| **M** | 06:00〜11:59 | Morning（朝） |
| **D** | 12:00〜17:59 | Day（昼） |
| **N** | 18:00〜23:59 | Night（夜） |
| **B** | 00:00〜05:59 | Brightning（深夜） |

---

## [5. FILE HIERARCHY EXAMPLES]

```
docs/
 ├─ ai_output_rules.md   ← 出力ルール（How）
 └─ ai_output_format.md  ← 出力構造仕様（What）
```

---

## [6. VERSIONING]

- 本テンプレート自体の更新も `[UPDATE LOG]` に従う。  
- バージョン番号は `vX.Y` で管理する。  
- 修正単位の目安：  
  - 軽微（文言調整） → +0.01  
  - 部分更新 → +0.1  
  - 体系変更 → +1.0  

---

## [UPDATE LOG]
- **2025-10-09B**
  - 初版作成。`ai_output_rules.md` との役割分離を定義。  
  - M/D/N/B 時間帯コード表記を正式採用。
