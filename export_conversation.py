"""
把 Claude Code 對話 (JSONL) 轉成 Markdown 檔

用法：
    python export_conversation.py <jsonl檔路徑> [輸出檔名.md]

範例：
    python export_conversation.py "C:/Users/maxbb/.claude/projects/d--Algaeorithm-pilot-backend/8d23305e-c1c4-4d04-91da-2a20bda0de7c.jsonl"
"""
import json
import sys
from pathlib import Path


def extract_text(content):
    """從 message.content 提取純文字（可能是 str 或 list[dict]）。"""
    if isinstance(content, str):
        return content
    if isinstance(content, list):
        parts = []
        for item in content:
            if isinstance(item, dict):
                if item.get("type") == "text":
                    parts.append(item.get("text", ""))
                elif item.get("type") == "tool_use":
                    name = item.get("name", "?")
                    parts.append(f"\n> 🔧 *[使用工具: {name}]*\n")
                elif item.get("type") == "tool_result":
                    parts.append("\n> 📋 *[工具執行結果]*\n")
                elif item.get("type") == "image":
                    parts.append("\n> 🖼️ *[圖片]*\n")
        return "\n".join(parts)
    return ""


def should_skip(entry):
    """跳過不需要的訊息。"""
    t = entry.get("type")
    if t in ("system", "queue-operation"):
        return True
    if entry.get("isMeta"):
        return True
    msg = entry.get("message", {})
    if isinstance(msg, dict):
        content = msg.get("content", "")
        if isinstance(content, str):
            # 跳過 slash command 系統訊息
            if content.startswith("<command-name>") or content.startswith("<local-command"):
                return True
            # 跳過純粹的 system-reminder
            if content.startswith("<system-reminder>") and len(content) < 300:
                return True
    return False


def format_message(entry):
    """把一筆訊息格式化成 Markdown。"""
    msg_type = entry.get("type", "unknown")
    msg = entry.get("message", {})
    content = msg.get("content", "") if isinstance(msg, dict) else ""
    text = extract_text(content)

    if not text.strip():
        return None

    timestamp = entry.get("timestamp", "")[:19].replace("T", " ")

    if msg_type == "user":
        return f"---\n\n### 👤 使用者 `{timestamp}`\n\n{text}\n"
    elif msg_type == "assistant":
        return f"\n### 🤖 Claude `{timestamp}`\n\n{text}\n"
    return None


def convert(jsonl_path, md_path):
    jsonl_path = Path(jsonl_path)
    md_path = Path(md_path)

    with open(jsonl_path, "r", encoding="utf-8") as f:
        lines = f.readlines()

    messages = []
    for line in lines:
        try:
            entry = json.loads(line)
        except json.JSONDecodeError:
            continue
        if should_skip(entry):
            continue
        formatted = format_message(entry)
        if formatted:
            messages.append(formatted)

    header = f"# Claude Code 對話紀錄\n\n來源檔案：`{jsonl_path.name}`\n\n共 {len(messages)} 則訊息\n\n"

    with open(md_path, "w", encoding="utf-8") as f:
        f.write(header)
        f.write("\n".join(messages))

    print(f"[OK] exported to: {md_path}")
    print(f"     messages: {len(messages)}")
    print(f"     size: {md_path.stat().st_size / 1024:.1f} KB")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    jsonl_path = sys.argv[1]
    md_path = sys.argv[2] if len(sys.argv) > 2 else Path(jsonl_path).with_suffix(".md").name

    convert(jsonl_path, md_path)
