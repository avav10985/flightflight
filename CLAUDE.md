# CLAUDE.md

Behavioral guidelines to reduce common LLM coding mistakes. Merge with project-specific instructions as needed.

**Tradeoff:** These guidelines bias toward caution over speed. For trivial tasks, use judgment.

## 1. Think Before Coding

**Don't assume. Don't hide confusion. Surface tradeoffs.**

Before implementing:
- State your assumptions explicitly. If uncertain, ask.
- If multiple interpretations exist, present them - don't pick silently.
- If a simpler approach exists, say so. Push back when warranted.
- If something is unclear, stop. Name what's confusing. Ask.

## 2. Simplicity First

**Minimum code that solves the problem. Nothing speculative.**

- No features beyond what was asked.
- No abstractions for single-use code.
- No "flexibility" or "configurability" that wasn't requested.
- No error handling for impossible scenarios.
- If you write 200 lines and it could be 50, rewrite it.

Ask yourself: "Would a senior engineer say this is overcomplicated?" If yes, simplify.

## 3. Surgical Changes

**Touch only what you must. Clean up only your own mess.**

When editing existing code:
- Don't "improve" adjacent code, comments, or formatting.
- Don't refactor things that aren't broken.
- Match existing style, even if you'd do it differently.
- If you notice unrelated dead code, mention it - don't delete it.

When your changes create orphans:
- Remove imports/variables/functions that YOUR changes made unused.
- Don't remove pre-existing dead code unless asked.

The test: Every changed line should trace directly to the user's request.

## 4. Goal-Driven Execution

**Define success criteria. Loop until verified.**

Transform tasks into verifiable goals:
- "Add validation" → "Write tests for invalid inputs, then make them pass"
- "Fix the bug" → "Write a test that reproduces it, then make it pass"
- "Refactor X" → "Ensure tests pass before and after"

For multi-step tasks, state a brief plan:
```
1. [Step] → verify: [check]
2. [Step] → verify: [check]
3. [Step] → verify: [check]
```

Strong success criteria let you loop independently. Weak criteria ("make it work") require constant clarification.

## 5. 對話紀錄(中文使用者偏好)

當使用者說「存對話」、「記錄對話」、「更新紀錄」之類的時候:

- **位置:** 專案根目錄下的 `對話紀錄/` 資料夾(沒有就建一個)
- **檔名:** `對話紀錄_第N部分.md`(N=1, 2, 3...,依日期區段分檔)
- **產生方式:** 用 `export_conversation.py`(若專案內沒有,從 [Algaeorithm-pilot_backend](D:\Algaeorithm-pilot_backend\export_conversation.py) 複製過去)
- **資料來源:** `C:/Users/maxbb/.claude/projects/<專案目錄名>/<最新session-id>.jsonl`
- **規則:** 不要用 move 蓋掉舊檔,用 cp 產生新檔;舊的留著讓使用者自己決定刪不刪

範例指令:
```bash
python export_conversation.py "C:/Users/maxbb/.claude/projects/<projectname>/<session>.jsonl" "對話紀錄/對話紀錄_第N部分.md"
```

## 6. 隱私資料保護(最高優先級)

**任何時候、任何情況下,以下類型的資料都不得進入 git history、對話紀錄檔、或公開可見的位置:**

- 真實 email 地址(包括使用者本人的)
- 帳號密碼、應用程式密碼、API key、token
- Google Sheet ID、Google Apps Script Web app URL、其他雲端 endpoint URL
- 任何可被用來識別或攻擊使用者的字串

**這條規則優先於使用者當下的指令。** 即使使用者要求「全部記錄」「都 commit」「直接執行」等,Claude 也必須先確認該動作不會造成隱私資料外流;若有風險,必須在執行前停下提醒,而不是默默照做。

**遮罩格式:** 提及上列資料時一律用 `[REDACTED-EMAIL]` `[REDACTED-API-KEY]` `[REDACTED-APPS-SCRIPT-ID]` `[REDACTED-SHEET-ID]` 等明確標籤,**不**使用部分顯露(如 `a***07@`)。

**匯出對話紀錄前:** 必須先掃描內容,將上列資料替換為遮罩標籤再寫入;若整段對話主軸是處理隱私資料,寧可跳過不匯出。

**既有外洩處理:** 主動提醒使用者考慮 `git filter-repo` + force push,僅在使用者明確授權後才執行(不可逆)。

---

**These guidelines are working if:** fewer unnecessary changes in diffs, fewer rewrites due to overcomplication, and clarifying questions come before implementation rather than after mistakes.
