"""
去除對話紀錄第 2、第 3 部分中與前一份重疊的內容
"""
import re
from pathlib import Path


def trim_after(md_path: Path, cutoff: str):
    """
    讀 md_path，把所有 timestamp <= cutoff 的訊息區塊刪掉。
    保留檔頭（前面的標題段）+ 所有 timestamp > cutoff 的訊息。
    cutoff 格式：'YYYY-MM-DD HH:MM:SS'
    """
    text = md_path.read_text(encoding='utf-8')

    # 取檔頭（第一個 "### 👤 使用者" 或 "### 🤖 Claude" 之前的部分）
    header_match = re.search(r'\n---\n\n### 👤 使用者 `', text)
    if not header_match:
        header_match = re.search(r'\n### 🤖 Claude `', text)
    header = text[:header_match.start()] if header_match else ''

    # 用 timestamp 切分為訊息區塊
    # 訊息區塊起點：'---\n\n### 👤 使用者 `TS`' 或 '\n### 🤖 Claude `TS`'
    pattern = re.compile(
        r'(?:\n?---\n\n### 👤 使用者|\n### 🤖 Claude) `(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2})`',
        re.MULTILINE
    )

    # 找出所有 match 的起點 + timestamp
    positions = [(m.start(), m.group(1)) for m in pattern.finditer(text)]
    positions.append((len(text), None))   # 加結尾哨兵

    kept_blocks = []
    for i in range(len(positions) - 1):
        start = positions[i][0]
        end = positions[i + 1][0]
        ts = positions[i][1]
        if ts and ts > cutoff:
            kept_blocks.append(text[start:end])

    # 重組
    new_content = header.rstrip() + '\n' + ''.join(kept_blocks)
    # 更新檔頭內的「共 X 則訊息」
    new_count = len(kept_blocks)
    new_content = re.sub(r'共 \d+ 則訊息', f'共 {new_count} 則訊息', new_content)

    return new_content, new_count


def main():
    base = Path('對話紀錄')

    # ---- 處理第 2 部分 ----
    p1 = base / '對話紀錄_第1部分.md'
    p2 = base / '對話紀錄_第2部分.md'
    p3 = base / '對話紀錄_第3部分.md'

    # 找第 1 部分最後 timestamp
    p1_text = p1.read_text(encoding='utf-8')
    p1_timestamps = re.findall(r'`(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2})`', p1_text)
    cutoff_for_p2 = p1_timestamps[-1]
    print(f'第 1 部分最後 timestamp: {cutoff_for_p2}')

    # 修剪第 2 部分
    new_p2, new_p2_count = trim_after(p2, cutoff_for_p2)
    p2.write_text(new_p2, encoding='utf-8')
    print(f'第 2 部分已修剪 → 剩 {new_p2_count} 則')

    # 找新的第 2 部分最後 timestamp
    p2_timestamps = re.findall(r'`(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2})`', new_p2)
    cutoff_for_p3 = p2_timestamps[-1] if p2_timestamps else cutoff_for_p2
    print(f'第 2 部分最後 timestamp: {cutoff_for_p3}')

    # 修剪第 3 部分
    new_p3, new_p3_count = trim_after(p3, cutoff_for_p3)
    p3.write_text(new_p3, encoding='utf-8')
    print(f'第 3 部分已修剪 → 剩 {new_p3_count} 則')

    # 統計
    print('\n=== 結果 ===')
    for p in [p1, p2, p3]:
        size_kb = p.stat().st_size / 1024
        ts_list = re.findall(r'`(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2})`',
                             p.read_text(encoding='utf-8'))
        print(f'{p.name}: {len(ts_list)} 則, {size_kb:.1f} KB, '
              f'{ts_list[0] if ts_list else "?"} ~ {ts_list[-1] if ts_list else "?"}')


if __name__ == '__main__':
    main()
