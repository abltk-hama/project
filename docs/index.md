# 🧭 Project Documentation Index
\---

## [PURPOSE]
このページは、**プロジェクト全体のドキュメント構造とAI関連仕様への導線**をまとめた  
「人間開発者向けポータル」です。  
ここを起点に、開発仕様・出力ルール・ディレクトリ構成をすばやく参照できます。

---

## [DOCUMENT TREE]

```
project/
 ├─ docs/
 │   ├─ index.md              ← ← このファイル（人間向けポータル）
 │   ├─ STRUCTURE.md          ← 階層構造と依存関係マップ
 │   ├─ ai_output_rules.md    ← AI出力ルール（How）
 │   └─ ai_output_format.md   ← AI出力形式仕様（What）
```

---

## [AI DOCUMENTATION LINKS]

| 分類 | ファイル | 内容概要 | 想定対象 |
|------|-----------|-----------|-----------|
| 🧩 **出力ルール** | [ai_output_rules.md](./ai_output_rules.md) | AIが出力時に守る行動・安全・構文ルールを定義（*How*） | AI & 開発者 |
| 🧱 **出力形式仕様** | [ai_output_format.md](./ai_output_format.md) | 生成物（md, code, json等）の章立て・表記ルール（*What*） | AI & 開発者 |
| 🧭 **階層構造定義** | [STRUCTURE.md](./STRUCTURE.md) | ディレクトリ構成、依存関係、役割マッピングを記述 | AI & 開発者 |
| ⚙️ **制御理論仕様** | [control_theory.md](./control_theory.md) | Pose Regulation・Lyapunov制御・PSO適応制御の設計指針 | 開発者 |
| 🚗 **物理モデル仕様** | [models.md](./models.md) | 車両運動モデルとRealVehicleインターフェースの統合 | 開発者 |
| 🧮 **シミュレーション仕様** | [sim.md](./sim.md) | 仮想・実機統合シミュレーションの構成と挙動確認方針 | 開発者 |
| 📈 **経路データ仕様** | [trajectory_schema.md](./trajectory_schema.md) | JSON形式の経路定義およびExcel管理との整合性 | 開発者 |

---

## [AI BEHAVIOR GUIDELINES]

> 本プロジェクトにおけるChatGPT（AI）の行動仕様・補助ルールは、  
> ChatGPT プロジェクト設定内の **「AI行動仕様書（v0.x）」** に準拠する。  
> その内容は [ai_output_rules.md](./ai_output_rules.md) および  
> [ai_output_format.md](./ai_output_format.md) に同期されている。

AIの行動仕様は次の原則に基づく：
- STRUCTURE.md に定義された階層を常に参照する。  
- 曖昧な指示は、1行仮定を明示した上で安全補完を行う。  
- 出力ログの日付は「M/D/N/B」時間帯コード方式で統一。  
- DeepResearch推奨判定は対話内容・規模に応じて動的に行う。  

---



## [UPDATE LOG]
- **2025-10-09N**
  - 初版作成。docs全体のナビゲーションを統合。
  - AI出力ルール（How）と形式仕様（What）のリンクを整備。
- **2025-10-10B**
  - 行動仕様書との同期運用方針を追加。
  - M/D/N/B表記方式を正式導入。
