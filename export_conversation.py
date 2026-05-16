"""
把 Claude Code 對話 (JSONL) 轉成 Markdown 檔

用法：
    python export_conversation.py <jsonl檔路徑> [輸出檔名.md] [--clean]

選項：
    --clean    去除工具呼叫、圖片佔位符、IDE 訊息、system-reminder 等雜訊
               讓我之後讀紀錄時更有效率（只看真正的對話脈絡）

範例：
    python export_conversation.py session.jsonl 紀錄.md --clean
"""
import json
import re
import sys
from pathlib import Path

# 全域旗標：是否啟用清潔模式
CLEAN_MODE = False


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
                    if not CLEAN_MODE:
                        name = item.get("name", "?")
                        parts.append(f"\n> 🔧 *[使用工具: {name}]*\n")
                elif item.get("type") == "tool_result":
                    if not CLEAN_MODE:
                        parts.append("\n> 📋 *[工具執行結果]*\n")
                elif item.get("type") == "image":
                    if not CLEAN_MODE:
                        parts.append("\n> 🖼️ *[圖片]*\n")
        return "\n".join(parts)
    return ""


def strip_noise(text):
    """清潔模式下移除 IDE / system 標籤包圍的雜訊段落。"""
    if not CLEAN_MODE or not text:
        return text
    # 移除 <ide_opened_file>...</ide_opened_file>
    text = re.sub(r"<ide_opened_file>.*?</ide_opened_file>", "", text, flags=re.DOTALL)
    # 移除 <ide_selection>...</ide_selection>
    text = re.sub(r"<ide_selection>.*?</ide_selection>", "", text, flags=re.DOTALL)
    # 移除 <system-reminder>...</system-reminder>
    text = re.sub(r"<system-reminder>.*?</system-reminder>", "", text, flags=re.DOTALL)
    # 移除 <command-name>...</command-name>
    text = re.sub(r"<command-name>.*?</command-name>", "", text, flags=re.DOTALL)
    # 移除 <local-command-stdout>...</local-command-stdout> 等
    text = re.sub(r"<local-command-[a-z]+>.*?</local-command-[a-z]+>", "", text, flags=re.DOTALL)
    # 移除空白段落，但保留段落結構
    text = re.sub(r"\n{3,}", "\n\n", text)
    return text.strip()


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
            # 清潔模式：跳過 content 整段被剝光只剩雜訊的訊息
            if CLEAN_MODE:
                stripped = strip_noise(content)
                if not stripped:
                    return True
    return False


def format_message(entry):
    """把一筆訊息格式化成 Markdown。"""
    msg_type = entry.get("type", "unknown")
    msg = entry.get("message", {})
    content = msg.get("content", "") if isinstance(msg, dict) else ""
    text = extract_text(content)
    text = strip_noise(text)

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

    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    flags = [a for a in sys.argv[1:] if a.startswith("--")]

    if "--clean" in flags:
        globals()["CLEAN_MODE"] = True

    jsonl_path = args[0]
    md_path = args[1] if len(args) > 1 else Path(jsonl_path).with_suffix(".md").name

    convert(jsonl_path, md_path)
