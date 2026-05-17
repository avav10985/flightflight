"""
電源擴充板 PCB 配置圖（市售模組風格）
- 綠色 FR4 + 白色絲印 + 金色焊盤
- 元件以真實外觀繪製
- 預留 I²S 麥克風擴充接腳
"""
import matplotlib.pyplot as plt
from matplotlib.patches import (Rectangle, Circle, FancyBboxPatch, Polygon,
                                Wedge, Ellipse)
from matplotlib.path import Path
import matplotlib.patches as mp
import matplotlib

matplotlib.rcParams['font.family'] = ['Microsoft JhengHei', 'Arial', 'sans-serif']
matplotlib.rcParams['axes.unicode_minus'] = False

fig, ax = plt.subplots(figsize=(15, 11), dpi=140)
ax.set_xlim(0, 24)
ax.set_ylim(0, 17)
ax.set_aspect('equal')
ax.axis('off')
fig.patch.set_facecolor('#E0E0E0')   # 整體底灰色，凸顯 PCB

# ===== 配色（仿真實 PCB）=====
PCB_GREEN = '#0E5132'        # 深綠 FR4
PCB_GREEN_LT = '#137A48'     # 略亮綠（高光）
SILK = '#F5F5F0'             # 白色絲印
SILK_DIM = '#C8C8B8'         # 暗一點的絲印
GOLD = '#E4B83C'             # ENIG 金色焊盤
GOLD_DARK = '#A07F1C'        # 金色焊盤陰影
TRACE_RED = '#FF5252'        # 5V 銅軌（示意，實際藏在綠漆下）
TRACE_BLK = '#1A1A1A'        # GND 銅軌
SOLDERMASK = '#0A3D27'       # 銅軌被綠漆覆蓋後的顏色

# ===== 標題 =====
ax.text(12, 16.4, '電源分配板 PCB 配置圖', ha='center',
        fontsize=24, fontweight='bold', color='#222')
ax.text(12, 15.85, 'FlightFlight v1.0  ·  俯視圖  ·  尺寸 50 × 80 mm',
        ha='center', fontsize=11, color='#666', style='italic')


# =================================================================
# PCB 板形（圓角矩形 + 四角固定孔）
# =================================================================
BX, BY, BW, BH = 2.5, 1.5, 19, 13.5

# 板面（深綠主體）
ax.add_patch(FancyBboxPatch(
    (BX, BY), BW, BH,
    boxstyle="round,pad=0.0,rounding_size=0.35",
    linewidth=0, facecolor=PCB_GREEN, zorder=1
))
# 板面邊緣亮色（仿絲印曝光的暈光）
ax.add_patch(FancyBboxPatch(
    (BX + 0.1, BY + 0.1), BW - 0.2, BH - 0.2,
    boxstyle="round,pad=0.0,rounding_size=0.3",
    linewidth=0.8, edgecolor='#0A3D27', facecolor='none', zorder=2
))
# 銅鋪地（淺斑點質感）
import numpy as np
np.random.seed(0)
for _ in range(280):
    rx = BX + 0.2 + np.random.random() * (BW - 0.4)
    ry = BY + 0.2 + np.random.random() * (BH - 0.4)
    rs = 0.025 + np.random.random() * 0.05
    ax.add_patch(Circle((rx, ry), rs, color='#0A3D27', alpha=0.4, zorder=1.5))

# 4 個固定孔（金色環）
for hx, hy in [(BX + 0.55, BY + 0.55), (BX + BW - 0.55, BY + 0.55),
               (BX + 0.55, BY + BH - 0.55), (BX + BW - 0.55, BY + BH - 0.55)]:
    ax.add_patch(Circle((hx, hy), 0.28, color=GOLD, zorder=3))
    ax.add_patch(Circle((hx, hy), 0.16, color='#1A1A1A', zorder=4))


# =================================================================
# 公用繪製函式
# =================================================================
def smt_pad(x, y, w=0.35, h=0.5, label='', label_offset=(0, 0.35), pin1=False):
    """金色 THT 焊盤（圓角矩形 + 鍍金 + 中央孔）"""
    if pin1:
        # 方形焊盤代表 pin 1
        ax.add_patch(Rectangle((x - w/2, y - h/2), w, h,
                               facecolor=GOLD, edgecolor=GOLD_DARK, linewidth=0.5, zorder=5))
    else:
        ax.add_patch(FancyBboxPatch(
            (x - w/2, y - h/2), w, h,
            boxstyle="round,pad=0,rounding_size=0.1",
            facecolor=GOLD, edgecolor=GOLD_DARK, linewidth=0.5, zorder=5))
    # 焊盤中央孔
    ax.add_patch(Circle((x, y), 0.08, color='#1A1A1A', zorder=6))
    if label:
        lx, ly = label_offset
        ax.text(x + lx, y + ly, label, ha='center', va='center', fontsize=7,
                color=SILK, fontweight='bold', zorder=10)


def pin_header(x, y, pins, label='', orient='V', label_pos='top'):
    """2.54mm 排針（黑塑膠座 + 金色針）"""
    n = len(pins)
    pitch = 0.5
    if orient == 'V':
        # 黑色塑膠座
        ax.add_patch(FancyBboxPatch(
            (x - 0.22, y - 0.25), 0.44, pitch * (n - 1) + 0.5,
            boxstyle="round,pad=0.0,rounding_size=0.05",
            facecolor='#1A1A1A', edgecolor='#000', linewidth=0.6, zorder=4))
        # 針 + 焊盤
        for i, p in enumerate(pins):
            py = y + i * pitch
            # 焊盤（pin1 為方形）
            smt_pad(x, py, w=0.32, h=0.32, pin1=(i == 0))
            # 針頭（凸出來的金屬）
            ax.add_patch(Rectangle((x - 0.06, py - 0.06), 0.12, 0.12,
                                   facecolor='#FFD700', edgecolor='#888',
                                   linewidth=0.3, zorder=7))
            # 腳位名稱（絲印）
            ax.text(x + 0.42, py, p, ha='left', va='center',
                    fontsize=7.5, color=SILK, fontweight='bold', zorder=10)
    else:  # 水平
        ax.add_patch(FancyBboxPatch(
            (x - 0.25, y - 0.22), pitch * (n - 1) + 0.5, 0.44,
            boxstyle="round,pad=0.0,rounding_size=0.05",
            facecolor='#1A1A1A', edgecolor='#000', linewidth=0.6, zorder=4))
        for i, p in enumerate(pins):
            px = x + i * pitch
            smt_pad(px, y, w=0.32, h=0.32, pin1=(i == 0))
            ax.add_patch(Rectangle((px - 0.06, y - 0.06), 0.12, 0.12,
                                   facecolor='#FFD700', edgecolor='#888',
                                   linewidth=0.3, zorder=7))
            ax.text(px, y + 0.42, p, ha='center', va='bottom',
                    fontsize=7.5, color=SILK, fontweight='bold', zorder=10)

    # 元件參考標籤
    if label and label_pos == 'left':
        ly_center = y + pitch * (n - 1) / 2 if orient == 'V' else y
        ax.text(x - 0.55, ly_center, label, ha='right', va='center',
                fontsize=9, color=SILK, fontweight='bold', zorder=10)
    elif label and label_pos == 'right':
        ly_center = y + pitch * (n - 1) / 2 if orient == 'V' else y
        ax.text(x + 1.2, ly_center, label, ha='left', va='center',
                fontsize=9, color=SILK, fontweight='bold', zorder=10)
    elif label and label_pos == 'top':
        lx_center = x + pitch * (n - 1) / 2 if orient == 'H' else x
        ly = y + (pitch * n) if orient == 'V' else y + 0.7
        ax.text(lx_center, ly, label, ha='center', va='bottom',
                fontsize=9, color=SILK, fontweight='bold', zorder=10)


def electrolytic_cap(x, y, value='', ref='', diameter=0.85):
    """電解電容（圓桶俯視，有極性條 + 十字紋）"""
    # 焊盤（兩腳）
    smt_pad(x - 0.4, y - 0.65, w=0.3, h=0.3, pin1=True)   # − 腳
    smt_pad(x + 0.4, y - 0.65, w=0.3, h=0.3)              # + 腳
    # 電容本體圓
    ax.add_patch(Circle((x, y), diameter, facecolor='#0D47A1',
                        edgecolor='#000', linewidth=1, zorder=8))
    ax.add_patch(Circle((x, y), diameter * 0.88, facecolor='#1565C0',
                        edgecolor='none', zorder=9))
    # 頂部十字紋（電解電容典型外觀）
    ax.plot([x - diameter * 0.5, x + diameter * 0.5], [y, y],
            color='#0D47A1', linewidth=1.5, zorder=10)
    ax.plot([x, x], [y - diameter * 0.5, y + diameter * 0.5],
            color='#0D47A1', linewidth=1.5, zorder=10)
    # 負極白條
    ax.add_patch(Wedge((x, y), diameter, 100, 260,
                       facecolor='white', alpha=0.85, zorder=9.5))
    ax.add_patch(Wedge((x, y), diameter, 100, 260,
                       facecolor='#1565C0', alpha=0.0,
                       edgecolor='none', zorder=10))
    ax.text(x - diameter * 0.7, y, '−', fontsize=14, color='#0D47A1',
            ha='center', va='center', fontweight='bold', zorder=11)
    # 元件值（絲印）
    ax.text(x, y - diameter - 0.6, value, ha='center', fontsize=8,
            color=SILK, fontweight='bold', zorder=10)
    # 參考編號（絲印）
    ax.text(x, y + diameter + 0.25, ref, ha='center', fontsize=9,
            color=SILK, fontweight='bold', zorder=10)


def ceramic_cap(x, y, value='', ref=''):
    """陶瓷電容（小米色長方塊俯視 + 兩腳）"""
    smt_pad(x - 0.35, y, w=0.28, h=0.28, pin1=True)
    smt_pad(x + 0.35, y, w=0.28, h=0.28)
    ax.add_patch(Rectangle((x - 0.25, y - 0.17), 0.5, 0.34,
                           facecolor='#E4D78C', edgecolor='#9C7F00',
                           linewidth=0.8, zorder=8))
    ax.text(x, y - 0.4, value, ha='center', fontsize=7,
            color=SILK, fontweight='bold', zorder=10)
    ax.text(x, y + 0.32, ref, ha='center', fontsize=8,
            color=SILK, fontweight='bold', zorder=10)


def led(x, y, ref=''):
    """LED 指示燈（紅色 + 兩腳）"""
    smt_pad(x - 0.25, y, w=0.25, h=0.25, pin1=True)  # 陽極
    smt_pad(x + 0.25, y, w=0.25, h=0.25)             # 陰極
    ax.add_patch(Circle((x, y), 0.22, facecolor='#E74C3C',
                        edgecolor='#922B21', linewidth=0.8, zorder=8))
    ax.add_patch(Circle((x, y), 0.16, facecolor='#FF6B5B',
                        edgecolor='none', zorder=9))
    # 高光
    ax.add_patch(Circle((x - 0.06, y + 0.06), 0.05, facecolor='white',
                        alpha=0.8, zorder=10))
    ax.text(x, y - 0.45, 'PWR', ha='center', fontsize=6.5,
            color=SILK, zorder=10)
    ax.text(x, y + 0.4, ref, ha='center', fontsize=7.5,
            color=SILK, fontweight='bold', zorder=10)


def resistor(x, y, value='', ref=''):
    """色環電阻（俯視長條 + 兩腳）"""
    smt_pad(x - 0.4, y, w=0.25, h=0.25, pin1=True)
    smt_pad(x + 0.4, y, w=0.25, h=0.25)
    # 電阻本體（米色）
    ax.add_patch(Rectangle((x - 0.3, y - 0.1), 0.6, 0.2,
                           facecolor='#F5DEB3', edgecolor='#8B6F47',
                           linewidth=0.7, zorder=8))
    # 色環（5V → LED 限流電阻通常 1k Ω = 棕黑紅）
    for col, dx in [('#8B4513', -0.18), ('#000', -0.12), ('#E74C3C', 0.05), ('#FFD700', 0.18)]:
        ax.plot([x + dx, x + dx], [y - 0.1, y + 0.1], color=col, linewidth=2.5, zorder=9)
    ax.text(x, y - 0.32, value, ha='center', fontsize=6.5,
            color=SILK, zorder=10)
    ax.text(x, y + 0.3, ref, ha='center', fontsize=7.5,
            color=SILK, fontweight='bold', zorder=10)


def trace(x1, y1, x2, y2, kind='5v', width=0.18):
    """銅軌（被綠漆覆蓋，畫成稍亮的色塊）"""
    color = SOLDERMASK if kind == 'covered' else (TRACE_RED if kind == '5v' else TRACE_BLK)
    if y1 == y2:
        ax.add_patch(Rectangle((min(x1, x2), y1 - width/2),
                               abs(x2 - x1), width,
                               facecolor=color, edgecolor='none',
                               alpha=0.7 if kind != 'covered' else 0.55, zorder=2.5))
    elif x1 == x2:
        ax.add_patch(Rectangle((x1 - width/2, min(y1, y2)),
                               width, abs(y2 - y1),
                               facecolor=color, edgecolor='none',
                               alpha=0.7 if kind != 'covered' else 0.55, zorder=2.5))


# =================================================================
# 元件配置
# =================================================================

# ----- 左側：4 顆 ESC 輸入排針（3-pin） -----
ESC_X = BX + 1.6
ESC_Y = [BY + 11.0, BY + 8.5, BY + 6.0, BY + 3.5]
for i, ey in enumerate(ESC_Y):
    pin_header(ESC_X, ey, ['SIG', '+5V', 'GND'],
               label=f'J{i+1}  ESC{i+1}', orient='V', label_pos='left')

# ----- 5V 主匯流條（紅色銅軌，從 ESC1 +5V 拉到右側） -----
BUS_5V_X = ESC_X + 1.5
BUS_5V_Y = BY + 11.5
trace(ESC_X, BUS_5V_Y - 0.5, BUS_5V_X, BUS_5V_Y - 0.5, kind='5v')   # ESC1 5V → 5V bus
trace(BUS_5V_X, BY + 2.0, BUS_5V_X, BUS_5V_Y - 0.5, kind='5v')      # 縱向主軌
trace(BUS_5V_X, BUS_5V_Y - 0.5, BX + BW - 1.6, BUS_5V_Y - 0.5, kind='5v')

# ----- GND 主匯流條 -----
BUS_GND_X = ESC_X + 2.2
trace(ESC_X, BY + 3.0, BUS_GND_X, BY + 3.0, kind='gnd')
trace(BUS_GND_X, BY + 2.5, BUS_GND_X, BUS_5V_Y - 1.0, kind='gnd')
# 各 ESC GND → GND 軌
for ey in ESC_Y:
    trace(ESC_X, ey, BUS_GND_X, ey, kind='gnd')
trace(BUS_GND_X, BY + 2.0, BX + BW - 1.6, BY + 2.0, kind='gnd')

# ----- 「NC」標記 ESC2~4 的 +5V 不接 -----
for ey in ESC_Y[1:]:
    ax.text(ESC_X + 0.7, ey, 'NC', fontsize=7, color='#FFD700',
            fontweight='bold', va='center', zorder=11,
            bbox=dict(boxstyle='round,pad=0.12', facecolor='#222',
                      edgecolor='#FFD700', linewidth=0.6))

# ----- 濾波電容（C1 電解 + C2 陶瓷）+ LED 指示燈 -----
electrolytic_cap(BX + 7.5, BY + 7.5, value='100µF\n16V', ref='C1')
ceramic_cap(BX + 9.5, BY + 7.5, value='0.1µF', ref='C2')

# C1、C2 接腳引線到銅軌
trace(BX + 7.1, BY + 6.8, BX + 7.1, BY + 2.0, kind='gnd', width=0.12)
trace(BX + 7.9, BY + 6.8, BX + 7.9, BUS_5V_Y - 0.5, kind='5v', width=0.12)
trace(BX + 9.1, BY + 7.5, BX + 9.1, BY + 2.0, kind='gnd', width=0.12)
trace(BX + 9.9, BY + 7.5, BX + 9.9, BUS_5V_Y - 0.5, kind='5v', width=0.12)

# LED 指示燈 + 限流電阻
led(BX + 11.5, BY + 8.5, ref='D1')
resistor(BX + 11.5, BY + 7.3, value='1kΩ', ref='R1')
trace(BX + 11.5, BY + 7.0, BX + 11.5, BY + 2.0, kind='gnd', width=0.12)
trace(BX + 11.5, BY + 8.7, BX + 11.5, BUS_5V_Y - 0.5, kind='5v', width=0.12)

# ----- 右側：5V/GND 輸出排針（OUT1~OUT4）+ I²S 麥克風預留（OUT5）-----
OUT_X = BX + BW - 1.6
out_specs = [
    (BY + 11.5, 'J5', 'OUT1 → ESP32 主控',  ['+5V', 'GND']),
    (BY + 9.5,  'J6', 'OUT2 → ESP32-CAM',  ['+5V', 'GND']),
    (BY + 7.5,  'J7', 'OUT3 → GPS 模組',    ['+5V', 'GND']),
    (BY + 5.5,  'J8', 'OUT4 → 蜂鳴器',      ['+5V', 'GND']),
]
for y, ref, txt, pins in out_specs:
    pin_header(OUT_X, y, pins, label=f'{ref}', orient='V', label_pos='right')
    trace(OUT_X, y + 0.5, OUT_X, BUS_5V_Y - 0.5, kind='5v', width=0.12)
    trace(OUT_X - 0.1, y, BUS_GND_X, y, kind='covered', width=0.08)
    trace(OUT_X, y, OUT_X, BY + 2.0, kind='gnd', width=0.12)
    # 去向標籤
    ax.text(BX + BW - 0.25, y + 0.25, txt, ha='right', va='center',
            fontsize=8, color=SILK, fontweight='bold', zorder=11,
            bbox=dict(boxstyle='round,pad=0.2', facecolor='#222',
                      edgecolor=SILK, linewidth=0.5))

# OUT5：I²S 麥克風預留（5-pin）
MIC_Y = BY + 2.7
pin_header(OUT_X, MIC_Y, ['VCC', 'GND', 'WS', 'SCK', 'SD'],
           label='J9', orient='V', label_pos='right')
ax.text(BX + BW - 0.25, MIC_Y + 1.6, 'OUT5 → I²S 麥克風\n(預留．未來擴充)',
        ha='right', va='center', fontsize=8, color='#FFD700',
        fontweight='bold', zorder=11,
        bbox=dict(boxstyle='round,pad=0.2', facecolor='#222',
                  edgecolor='#FFD700', linewidth=0.7))
# I²S 預留 VCC/GND 從電源軌取
trace(OUT_X, MIC_Y + 2.0, OUT_X, BUS_5V_Y - 0.5, kind='5v', width=0.1)
trace(OUT_X, MIC_Y + 1.5, BUS_GND_X, MIC_Y + 1.5, kind='covered', width=0.08)
# 訊號腳（WS/SCK/SD）為「無連接」，飛機端 ESP32 拉線過去
ax.text(OUT_X - 0.6, MIC_Y + 0.5, '訊號腳\n→ ESP32\n(自行飛線)',
        ha='right', va='center', fontsize=7, color=SILK_DIM,
        style='italic', zorder=10)

# ----- 板邊絲印：板名 + 版本 + 製造商標記 -----
ax.text(BX + 0.9, BY + BH - 0.45, 'POWER-BD v1.0',
        ha='left', va='top', fontsize=11, color=SILK,
        fontweight='bold', zorder=10)
ax.text(BX + 0.9, BY + BH - 0.85, 'FlightFlight 專案 / 2026',
        ha='left', va='top', fontsize=8.5, color=SILK_DIM, zorder=10)

# 中央絲印 logo
ax.text(BX + BW/2, BY + 5.0, 'FlightFlight',
        ha='center', va='center', fontsize=18, color=SILK_DIM,
        fontweight='bold', alpha=0.5, zorder=2.8, style='italic')

# Pin1 指示符號（每個 header 旁邊一個小白點）
for ey in ESC_Y:
    ax.add_patch(Circle((ESC_X - 0.35, ey), 0.08, color=SILK, zorder=11))
for y, _, _, _ in out_specs:
    ax.add_patch(Circle((OUT_X - 0.35, y + 0.5), 0.08, color=SILK, zorder=11))
ax.add_patch(Circle((OUT_X - 0.35, MIC_Y + 2.0), 0.08, color=SILK, zorder=11))

# ===== 底部圖例 / BoM =====
legend_x, legend_y = 2.7, 0.75
ax.text(legend_x, legend_y,
        '銅軌色：紅 = +5V    黑 = GND    暗綠 = 被綠漆覆蓋\n'
        '材料清單：C1 = 100µF/16V 電解  ·  C2 = 0.1µF 陶瓷  ·  R1 = 1kΩ  ·  D1 = 紅 LED 3mm  ·  排針間距 2.54mm',
        fontsize=9.5, color='#222', va='center')

plt.savefig('d:/無人機/flightflight/breakout_pcb.png',
            dpi=140, bbox_inches='tight', facecolor='#E0E0E0')
print("Saved: breakout_pcb.png")
