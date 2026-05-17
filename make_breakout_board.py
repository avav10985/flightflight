"""
洞洞板電源擴充板示意圖
- 從 ESC1 取 5V 給所有電子模組
- 4 顆 ESC GND 共地
- 100µF 電解 + 0.1µF 陶瓷濾波
"""
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle, FancyBboxPatch
import matplotlib

matplotlib.rcParams['font.family'] = ['Microsoft JhengHei', 'sans-serif']
matplotlib.rcParams['axes.unicode_minus'] = False

fig, ax = plt.subplots(figsize=(16, 10), dpi=130)
ax.set_xlim(0, 22)
ax.set_ylim(0, 14)
ax.set_aspect('equal')
ax.axis('off')
fig.patch.set_facecolor('white')

# 顏色
BOARD = '#D4A574'        # 洞洞板棕色
HOLE = '#8B6F47'         # 洞顏色
WIRE_5V = '#C0392B'      # 5V 線（紅）
WIRE_GND = '#000000'     # GND 線（黑）
WIRE_SIG = '#27AE60'     # 信號線
SOLDER = '#888888'       # 焊點
TEXT = '#111'

# ============ 標題 ============
ax.text(11, 13.5, '洞洞板 — 電源擴充板（5V/GND 分配）', ha='center',
        fontsize=20, fontweight='bold', color=TEXT)
ax.text(11, 13.0,
        '從 ESC1 紅線取 5V，分給 ESP32 / ESP32-CAM / GPS；4 顆 ESC 黑線共地',
        ha='center', fontsize=11, color='gray', style='italic')

# ============ 洞洞板背景 ============
BOARD_X, BOARD_Y = 3, 3
BOARD_W, BOARD_H = 16, 8
ax.add_patch(FancyBboxPatch(
    (BOARD_X, BOARD_Y), BOARD_W, BOARD_H,
    boxstyle="round,pad=0.05,rounding_size=0.2",
    linewidth=2, edgecolor='#6B5436', facecolor=BOARD, zorder=1
))

# 板上的洞（網格）
HOLE_PITCH = 0.4
for ix in range(int(BOARD_W / HOLE_PITCH) - 1):
    for iy in range(int(BOARD_H / HOLE_PITCH) - 1):
        hx = BOARD_X + 0.3 + ix * HOLE_PITCH
        hy = BOARD_Y + 0.3 + iy * HOLE_PITCH
        ax.add_patch(Circle((hx, hy), 0.06, color=HOLE, zorder=2))

# ============ 5V 主匯流條（紅線，貫穿） ============
BUS_5V_Y = 8.5
BUS_GND_Y = 5.5
ax.plot([BOARD_X + 0.3, BOARD_X + BOARD_W - 0.3], [BUS_5V_Y, BUS_5V_Y],
        color=WIRE_5V, linewidth=4, zorder=3)
ax.text(BOARD_X + BOARD_W - 0.3, BUS_5V_Y + 0.35, '5V 主匯流條',
        ha='right', fontsize=11, fontweight='bold', color=WIRE_5V, zorder=4)

# ============ GND 主匯流條（黑線，貫穿） ============
ax.plot([BOARD_X + 0.3, BOARD_X + BOARD_W - 0.3], [BUS_GND_Y, BUS_GND_Y],
        color=WIRE_GND, linewidth=4, zorder=3)
ax.text(BOARD_X + BOARD_W - 0.3, BUS_GND_Y - 0.55, 'GND 主匯流條',
        ha='right', fontsize=11, fontweight='bold', color=WIRE_GND, zorder=4)


def solder_pad(x, y, size=0.18):
    """畫一個焊點"""
    ax.add_patch(Circle((x, y), size, color=SOLDER, zorder=5))
    ax.add_patch(Circle((x, y), size * 0.6, color='#444', zorder=6))


def label_box(x, y, text, color=TEXT, fontsize=9, bg='white'):
    ax.text(x, y, text, ha='center', va='center', fontsize=fontsize, color=color,
            bbox=dict(boxstyle='round,pad=0.2', facecolor=bg,
                      edgecolor=color, linewidth=0.8), zorder=10)


# ============ ESC 1 ~ 4 的接入點（左側） ============
esc_y_positions = [10.5, 9.5, 7.5, 6.5]   # M1, M2, M3, M4
esc_names = ['ESC1 (M1 前左)', 'ESC2 (M2 前右)', 'ESC3 (M3 後左)', 'ESC4 (M4 後右)']

for i, (y, name) in enumerate(zip(esc_y_positions, esc_names)):
    # 從左邊進來的標籤
    ax.text(1.5, y, name, ha='right', fontsize=9, fontweight='bold')

    if i == 0:  # ESC1：紅線進 5V、黑線進 GND
        # 紅線
        ax.plot([1.8, BOARD_X], [y + 0.15, y + 0.15], color=WIRE_5V, linewidth=2.5, zorder=3)
        ax.plot([BOARD_X + 0.3, BOARD_X + 0.3, BOARD_X + 0.3],
                [y + 0.15, y + 0.15, BUS_5V_Y], color=WIRE_5V, linewidth=2.5, zorder=3)
        ax.plot([BOARD_X + 0.3], [BUS_5V_Y], 'o', color=WIRE_5V, markersize=8, zorder=6)
        solder_pad(BOARD_X + 0.3, y + 0.15)
        label_box(2.5, y + 0.4, '紅 +5V', color=WIRE_5V, fontsize=8)

        # 黑線
        ax.plot([1.8, BOARD_X], [y - 0.15, y - 0.15], color=WIRE_GND, linewidth=2.5, zorder=3)
        ax.plot([BOARD_X + 0.6, BOARD_X + 0.6, BOARD_X + 0.6],
                [y - 0.15, y - 0.15, BUS_GND_Y], color=WIRE_GND, linewidth=2.5, zorder=3)
        solder_pad(BOARD_X + 0.6, y - 0.15)
        label_box(2.5, y - 0.4, '黑 GND', color=WIRE_GND, fontsize=8)

        # 訊號線（白）→ 不進這塊板，直接走 ESP32
        ax.plot([1.8, 2.5], [y, y], color=WIRE_SIG, linewidth=2)
        ax.annotate('白：SIG → 直連 GPIO25\n(不進擴充板)',
                    xy=(2.5, y), xytext=(2.7, y),
                    fontsize=7, color=WIRE_SIG, va='center')

    else:  # ESC2/3/4：紅線包熱縮、黑線進 GND
        # 紅線（用虛線表示「包起來」）
        ax.plot([1.8, 2.3], [y + 0.15, y + 0.15], color=WIRE_5V,
                linewidth=2.5, linestyle='--', zorder=3)
        # 熱縮套標記
        ax.add_patch(Rectangle((2.2, y), 0.4, 0.3, color='#FFD700',
                               edgecolor='#B7950B', linewidth=1, zorder=4))
        ax.text(2.4, y + 0.5, '熱縮套\n(不接)', ha='center', fontsize=7,
                color='#B7950B', fontweight='bold')

        # 黑線進 GND
        ax.plot([1.8, BOARD_X], [y - 0.15, y - 0.15], color=WIRE_GND, linewidth=2.5, zorder=3)
        x_in = BOARD_X + 0.3 + (i + 1) * 0.4
        ax.plot([BOARD_X, x_in, x_in], [y - 0.15, y - 0.15, BUS_GND_Y],
                color=WIRE_GND, linewidth=2.5, zorder=3)
        solder_pad(x_in, BUS_GND_Y)
        label_box(2.5, y - 0.4, '黑 GND', color=WIRE_GND, fontsize=8)


# ============ 濾波電容 C1, C2 ============
# C1 = 100µF 電解（極性）
cap1_x = BOARD_X + 3.5
ax.plot([cap1_x, cap1_x], [BUS_5V_Y, BUS_5V_Y - 0.5], color=WIRE_5V, linewidth=2)
# 電容符號
ax.plot([cap1_x - 0.4, cap1_x + 0.4], [BUS_5V_Y - 0.5, BUS_5V_Y - 0.5], color='#222', linewidth=3)
ax.plot([cap1_x - 0.4, cap1_x + 0.4], [BUS_5V_Y - 0.8, BUS_5V_Y - 0.8], color='#222', linewidth=3)
ax.text(cap1_x - 0.6, BUS_5V_Y - 0.45, '+', fontsize=12, color='#222', fontweight='bold')
ax.plot([cap1_x, cap1_x], [BUS_5V_Y - 0.8, BUS_GND_Y], color=WIRE_GND, linewidth=2)
solder_pad(cap1_x, BUS_5V_Y)
solder_pad(cap1_x, BUS_GND_Y)
label_box(cap1_x, BUS_5V_Y - 1.55, 'C1\n100µF\n電解', fontsize=8, bg='#FFFACD')

# C2 = 0.1µF 陶瓷（無極性）
cap2_x = BOARD_X + 5.0
ax.plot([cap2_x, cap2_x], [BUS_5V_Y, BUS_5V_Y - 0.5], color=WIRE_5V, linewidth=2)
ax.plot([cap2_x - 0.3, cap2_x + 0.3], [BUS_5V_Y - 0.5, BUS_5V_Y - 0.5], color='#222', linewidth=3)
ax.plot([cap2_x - 0.3, cap2_x + 0.3], [BUS_5V_Y - 0.8, BUS_5V_Y - 0.8], color='#222', linewidth=3)
ax.plot([cap2_x, cap2_x], [BUS_5V_Y - 0.8, BUS_GND_Y], color=WIRE_GND, linewidth=2)
solder_pad(cap2_x, BUS_5V_Y)
solder_pad(cap2_x, BUS_GND_Y)
label_box(cap2_x, BUS_5V_Y - 1.55, 'C2\n0.1µF\n陶瓷', fontsize=8, bg='#FFFACD')

# ============ 5V 輸出端（右側） ============
output_y_positions = [10.0, 9.0, 8.0]
output_names = [
    ('→ ESP32 VIN', '紅', '黑'),
    ('→ ESP32-CAM 5V', '紅', '黑'),
    ('→ GPS VCC', '紅', '黑'),
]

for i, ((y, (name, p, n))) in enumerate(zip(output_y_positions, output_names)):
    # 從 5V 匯流條拉出
    x_out_5v = BOARD_X + BOARD_W - 1.5 - i * 1.0
    ax.plot([x_out_5v, x_out_5v], [BUS_5V_Y, y + 0.15], color=WIRE_5V, linewidth=2.5, zorder=3)
    ax.plot([x_out_5v, BOARD_X + BOARD_W], [y + 0.15, y + 0.15], color=WIRE_5V, linewidth=2.5, zorder=3)
    solder_pad(x_out_5v, BUS_5V_Y)

    # 從 GND 匯流條拉出
    x_out_gnd = x_out_5v + 0.3
    ax.plot([x_out_gnd, x_out_gnd], [BUS_GND_Y, y - 0.15], color=WIRE_GND, linewidth=2.5, zorder=3)
    ax.plot([x_out_gnd, BOARD_X + BOARD_W], [y - 0.15, y - 0.15], color=WIRE_GND, linewidth=2.5, zorder=3)
    solder_pad(x_out_gnd, BUS_GND_Y)

    # 輸出端標籤
    ax.text(BOARD_X + BOARD_W + 0.3, y, name, ha='left', va='center',
            fontsize=10, fontweight='bold')

# GND 額外輸出給 4 顆 ESC 已經接到 GND 匯流條，不重複拉

# ============ 標示說明 ============
note_x, note_y = 0.5, 2
ax.add_patch(Rectangle((note_x, note_y - 1.5), 21, 1.4,
                       linewidth=1, edgecolor='gray', facecolor='#F8F9FA', zorder=1))
ax.text(note_x + 0.3, note_y - 0.3, '◆ 焊接要點', fontsize=11, fontweight='bold', color='#C0392B')
notes = [
    '1. 5V 主匯流條：用粗銅線（22 AWG 以上）或多孔短路成一條，承受 1~2A',
    '2. GND 主匯流條：同上。所有零件 GND 都到這條，**飛機所有 GND 必須共地**',
    '3. C1（100µF）+ C2（0.1µF）並聯：吸收馬達啟動瞬間電壓抖動',
    '4. ESC1 紅線是「唯一電源來源」，ESC2/3/4 紅線**包熱縮套**完全不接',
    '5. 焊點要飽滿，飛機震動會讓虛焊脫落 → 炸機',
]
for i, n in enumerate(notes):
    ax.text(note_x + 0.5, note_y - 0.55 - i * 0.18, n, fontsize=9, color=TEXT)

# ============ 線色圖例（右下） ============
leg_x, leg_y = 18, 1.5
ax.add_patch(Rectangle((leg_x, leg_y - 0.8), 3.2, 1.1, linewidth=1, edgecolor='gray',
                       facecolor='white', alpha=0.95, zorder=1))
ax.text(leg_x + 0.1, leg_y + 0.1, '線色', fontsize=9, fontweight='bold', zorder=2)
for i, (c, lbl) in enumerate([(WIRE_5V, '5V (紅)'), (WIRE_GND, 'GND (黑)'), (WIRE_SIG, 'SIG (白)')]):
    yy = leg_y - 0.2 - i * 0.2
    ax.plot([leg_x + 0.2, leg_x + 0.7], [yy, yy], color=c, linewidth=2.5, zorder=2)
    ax.text(leg_x + 0.8, yy, lbl, fontsize=8.5, va='center', zorder=2)

plt.savefig('d:/無人機/flightflight/breakout_board.png',
            dpi=130, bbox_inches='tight', facecolor='white')
print("Saved: breakout_board.png")
