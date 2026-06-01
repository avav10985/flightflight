"""
手把 V2-A 主板(A 板)走線示意圖
順達 9cm × 15cm 洞洞板，橫向擺放
S3-N16R8 + NRF24 + buck + TFT/B 板對外排線
"""
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, FancyBboxPatch
import matplotlib
import numpy as np

matplotlib.rcParams['font.family'] = ['Microsoft JhengHei', 'sans-serif']
matplotlib.rcParams['axes.unicode_minus'] = False

fig, ax = plt.subplots(figsize=(18, 11), dpi=140)
ax.set_xlim(-0.6, 15.8)
ax.set_ylim(-0.6, 10.4)
ax.set_aspect('equal')
ax.axis('off')
fig.patch.set_facecolor('white')

# 顏色
PCB     = '#E8B57B'
PCB_EDGE = '#8B5A2B'
S3_BG   = '#1B1F23'
S3_FG   = '#FFFFFF'
PIN_HL  = '#FFD56B'
RAIL_3V3 = '#C0392B'
RAIL_5V  = '#E67E22'
RAIL_GND = '#222222'
SIG_TFT  = '#9B59B6'
SIG_NRF  = '#27AE60'
SIG_IN   = '#2980B9'
SIG_SD   = '#16A085'
COMP_BUCK = '#3498DB'
COMP_NRF  = '#27AE60'
COMP_TFT_HDR = '#9B59B6'
COMP_B_HDR   = '#E74C3C'

# 標題
ax.text(7.5, 9.9, '手把 V2-A 主板走線圖 ─ 順達 9×15cm 橫向',
        ha='center', fontsize=18, fontweight='bold', color='#1F2937')
ax.text(7.5, 9.5, 'ESP32-S3-N16R8 + NRF24 + buck;TFT/B 板用一般排線直接焊',
        ha='center', fontsize=10, color='gray', style='italic')

# 洞洞板背景
ax.add_patch(FancyBboxPatch((0, 0), 15, 9,
             boxstyle="round,pad=0.02,rounding_size=0.18",
             linewidth=2.2, edgecolor=PCB_EDGE, facecolor=PCB, alpha=0.55, zorder=1))
for x in np.arange(0.3, 14.8, 0.5):
    for y in np.arange(0.3, 8.8, 0.5):
        ax.plot(x, y, 'o', markersize=0.6, color=PCB_EDGE, alpha=0.35, zorder=1.5)

# 電源軌
for y, col, txt in [(8.78, RAIL_GND, 'GND 軌'), (8.50, RAIL_3V3, '3V3 軌'), (8.22, RAIL_5V, '5V 軌')]:
    ax.plot([0.3, 14.7], [y, y], '-', color=col, linewidth=2.4, zorder=3)
    ax.text(0.1, y, txt, ha='right', va='center', fontsize=8.5, color=col, fontweight='bold')

# S3 模組(垂直)
s3_x, s3_y, s3_w, s3_h = 6.6, 0.55, 2.0, 7.2
ax.add_patch(FancyBboxPatch((s3_x, s3_y), s3_w, s3_h,
             boxstyle="round,pad=0.02,rounding_size=0.08",
             linewidth=1.6, edgecolor='#000', facecolor=S3_BG, zorder=4))
ax.text(s3_x + s3_w/2, s3_y + s3_h - 0.35, 'ESP32-S3', ha='center', va='top',
        fontsize=10.5, fontweight='bold', color=S3_FG, zorder=5)
ax.text(s3_x + s3_w/2, s3_y + s3_h - 0.75, 'N16R8', ha='center', va='top',
        fontsize=8.5, color='#BBB', zorder=5)
ax.text(s3_x + s3_w/2, s3_y + 0.15, '◢ USB-C ×2 ◣', ha='center', va='bottom',
        fontsize=7.5, color='#888', zorder=5)
ax.text(s3_x + s3_w/2, s3_y + s3_h + 0.20, '▲ 天線', ha='center', va='bottom',
        fontsize=7.5, color='#666', zorder=5)

left_pins  = ['3V3','3V3','RST','4','5','6','7','15','16','17','18','8','3','46','9','10','11','12','13','14','5V','GND']
right_pins = ['GND','TX','RX','1','2','42','41','40','39','38','37','36','35','0','45','48','47','21','20','19','GND','3V3']

use_left = {
    '4':'roll', '5':'SW_A', '6':'SW_B', '7':'menu',
    '8':'肩L', '9':'肩R',
    '15':'TFT D4','16':'TFT D5','17':'TFT D6','18':'TFT D7',
    '11':'TFT D0','12':'TFT D1','13':'TFT D2','14':'TFT D3',
}
use_right = {
    '1':'throttle','2':'pitch',
    '42':'NRF CSN','41':'NRF CE','40':'MISO','39':'MOSI','38':'SCK',
    '0':'SD CS',
    '48':'TFT CS','47':'TFT WR','21':'TFT RS',
}

n_pins = 22
pin_spacing = s3_h / n_pins
left_pin_x  = s3_x
right_pin_x = s3_x + s3_w

def y_of(side, label):
    pins = left_pins if side == 'L' else right_pins
    i = pins.index(label)
    return s3_y + s3_h - (i + 0.5) * pin_spacing

# 畫 pin 點與標籤
for i, p in enumerate(left_pins):
    y = s3_y + s3_h - (i + 0.5) * pin_spacing
    ax.plot(left_pin_x, y, 'o', markersize=5, color=PIN_HL, markeredgecolor='black', linewidth=0.5, zorder=6)
    use = use_left.get(p, '')
    if use:
        ax.text(left_pin_x - 0.06, y, f'{p} {use}', ha='right', va='center',
                fontsize=7.8, color='#222', fontweight='bold', zorder=6)
    else:
        ax.text(left_pin_x - 0.06, y, p, ha='right', va='center',
                fontsize=7.2, color='#888', zorder=6)

for i, p in enumerate(right_pins):
    y = s3_y + s3_h - (i + 0.5) * pin_spacing
    ax.plot(right_pin_x, y, 'o', markersize=5, color=PIN_HL, markeredgecolor='black', linewidth=0.5, zorder=6)
    use = use_right.get(p, '')
    if use:
        ax.text(right_pin_x + 0.06, y, f'{p} {use}', ha='left', va='center',
                fontsize=7.8, color='#222', fontweight='bold', zorder=6)
    else:
        ax.text(right_pin_x + 0.06, y, p, ha='left', va='center',
                fontsize=7.2, color='#888', zorder=6)

# buck(左下)
bk_x, bk_y, bk_w, bk_h = 0.5, 1.4, 2.6, 2.0
ax.add_patch(FancyBboxPatch((bk_x, bk_y), bk_w, bk_h,
             boxstyle="round,pad=0.02,rounding_size=0.1",
             linewidth=1.5, edgecolor=COMP_BUCK, facecolor='#D6EAF8', zorder=4))
ax.text(bk_x + bk_w/2, bk_y + bk_h - 0.3, 'buck 降壓', ha='center',
        fontsize=10, fontweight='bold', color=COMP_BUCK)
ax.text(bk_x + bk_w/2, bk_y + bk_h - 0.75, 'MP1584 / LM2596', ha='center',
        fontsize=7.8, color='#555')
ax.text(bk_x + bk_w/2, bk_y + 0.5, '輸出調 5.0V\nOUT→5V 軌', ha='center',
        fontsize=8.5, color='#C0392B', fontweight='bold')

# 電池+開關
ax.add_patch(Rectangle((0.5, 0.3), 2.6, 0.9, linewidth=1.2, edgecolor='#7F8C8D',
             facecolor='#FDFDFD', zorder=4))
ax.text(0.5 + 1.3, 0.75, '電源開關 + 電池接頭(2S)', ha='center', va='center',
        fontsize=8, color='#333')

# buck → 5V 軌
ax.plot([bk_x + bk_w - 0.3, bk_x + bk_w - 0.3, 5.0], [bk_y + bk_h, 8.22, 8.22],
        '-', color=RAIL_5V, linewidth=2.0, zorder=2.5)

# NRF24(右上)
nrf_x, nrf_y, nrf_w, nrf_h = 11.5, 4.2, 3.0, 3.0
ax.add_patch(FancyBboxPatch((nrf_x, nrf_y), nrf_w, nrf_h,
             boxstyle="round,pad=0.02,rounding_size=0.1",
             linewidth=1.5, edgecolor=COMP_NRF, facecolor='#D5F5E3', zorder=4))
ax.text(nrf_x + nrf_w/2, nrf_y + nrf_h - 0.3, 'NRF24L01+PA/LNA', ha='center',
        fontsize=10, fontweight='bold', color=COMP_NRF)
ax.text(nrf_x + nrf_w/2, nrf_y + nrf_h - 0.75, '緊貼 GPIO 38~42', ha='center',
        fontsize=7.8, color='#555')
ax.text(nrf_x + nrf_w/2, nrf_y + nrf_h - 1.2, '+ 100µF + 0.1µF', ha='center',
        fontsize=8.5, color='#C0392B', fontweight='bold')
ax.text(nrf_x + nrf_w/2, nrf_y + 0.6, 'CE=41 CSN=42\nSCK=38 MOSI=39\nMISO=40', ha='center',
        va='center', fontsize=8, color='#333')

# TFT 對外排線(頂)
tft_hdr_x, tft_hdr_y, tft_hdr_w, tft_hdr_h = 3.6, 7.85, 7.8, 0.40
ax.add_patch(FancyBboxPatch((tft_hdr_x, tft_hdr_y), tft_hdr_w, tft_hdr_h,
             boxstyle="round,pad=0.01,rounding_size=0.05",
             linewidth=1.5, edgecolor=COMP_TFT_HDR, facecolor='#E8DAEF', zorder=4))
ax.text(tft_hdr_x + tft_hdr_w/2, tft_hdr_y + tft_hdr_h/2,
        'TFT 排線(~14 條,焊到 TFT 轉接小板)',
        ha='center', va='center', fontsize=10, fontweight='bold', color=COMP_TFT_HDR)

# B 板對外排線(底)
b_hdr_x, b_hdr_y, b_hdr_w, b_hdr_h = 3.6, 0.05, 7.5, 0.40
ax.add_patch(FancyBboxPatch((b_hdr_x, b_hdr_y), b_hdr_w, b_hdr_h,
             boxstyle="round,pad=0.01,rounding_size=0.05",
             linewidth=1.5, edgecolor=COMP_B_HDR, facecolor='#FADBD8', zorder=4))
ax.text(b_hdr_x + b_hdr_w/2, b_hdr_y + b_hdr_h/2,
        'B 板排線(10 條,焊到搖桿/按鈕板)',
        ha='center', va='center', fontsize=10, fontweight='bold', color=COMP_B_HDR)

# === 走線 ===
# TFT 資料 D0~D7(左側,向上)
tft_data_pins = [('15', 4.0), ('16', 4.2), ('17', 4.4), ('18', 4.6),
                 ('11', 4.8), ('12', 5.0), ('13', 5.2), ('14', 5.4)]
for p, x_via in tft_data_pins:
    y0 = y_of('L', p)
    ax.plot([s3_x, x_via, x_via], [y0, y0, tft_hdr_y],
            '-', color=SIG_TFT, linewidth=1.0, alpha=0.85, zorder=2.5)

# TFT 控制 RS/WR/CS(右側,跨上)
for p, x_via, dash in [('21', 11.2, True), ('47', 10.9, True), ('48', 10.6, True)]:
    y0 = y_of('R', p)
    ls = '--' if dash else '-'
    ax.plot([s3_x + s3_w, x_via, x_via], [y0, y0, tft_hdr_y],
            ls, color=SIG_TFT, linewidth=1.0, alpha=0.7, zorder=2.5)

# NRF24 SPI(右側,直接出去)
for p in ['38','39','40','41','42']:
    y0 = y_of('R', p)
    ax.plot([s3_x + s3_w, nrf_x], [y0, y0],
            '-', color=SIG_NRF, linewidth=1.3, alpha=0.9, zorder=2.5)

# SD CS(GPIO 0)
y0 = y_of('R', '0')
ax.plot([s3_x + s3_w, 11.0, 11.0, nrf_x + nrf_w + 0.3, nrf_x + nrf_w + 0.3],
        [y0, y0, nrf_y + nrf_h + 0.15, nrf_y + nrf_h + 0.15, nrf_y + nrf_h + 0.6],
        '-', color=SIG_SD, linewidth=1.1, alpha=0.85, zorder=2.5)
ax.text(nrf_x + nrf_w + 0.35, nrf_y + nrf_h + 0.7,
        'SD CS → TFT 卡座', fontsize=7.5, color=SIG_SD, fontweight='bold')

# 輸入(左側 4,5,6,7,8,9 向下)
for p, x_via in [('4', 5.6), ('5', 5.8), ('6', 6.0), ('7', 6.2),
                 ('8', 4.6), ('9', 4.4)]:
    y0 = y_of('L', p)
    ax.plot([s3_x, x_via, x_via], [y0, y0, b_hdr_y + b_hdr_h],
            '-', color=SIG_IN, linewidth=1.0, alpha=0.85, zorder=2.5)

# 輸入(右側 1, 2 跨下)
for p, x_via, dash in [('1', 10.3, True), ('2', 10.5, True)]:
    y0 = y_of('R', p)
    ls = '--' if dash else '-'
    ax.plot([s3_x + s3_w, x_via, x_via], [y0, y0, b_hdr_y + b_hdr_h],
            ls, color=SIG_IN, linewidth=1.0, alpha=0.7, zorder=2.5)

# === 圖例(右上) ===
legend_box = FancyBboxPatch((11.7, 7.4), 3.0, 0.75,
              boxstyle="round,pad=0.02,rounding_size=0.05",
              linewidth=1, edgecolor='#888', facecolor='#FAFAFA', zorder=5)
ax.add_patch(legend_box)
legend_items = [
    ('TFT 資料/控制', SIG_TFT),
    ('NRF24 SPI',     SIG_NRF),
    ('SD CS',         SIG_SD),
    ('輸入 8 線',     SIG_IN),
]
for i, (txt, c) in enumerate(legend_items):
    row, col = i // 2, i % 2
    yy = 8.0 - row * 0.32
    xx = 11.85 + col * 1.55
    ax.plot([xx, xx + 0.3], [yy, yy], '-', color=c, linewidth=2.2)
    ax.text(xx + 0.35, yy, txt, va='center', fontsize=8, color='#222', zorder=6)

# === 焊接重點(右下,NRF24 下方,完全不與其他元件重疊) ===
notes_x, notes_y, notes_w, notes_h = 11.5, 0.6, 3.0, 3.2
ax.add_patch(FancyBboxPatch((notes_x, notes_y), notes_w, notes_h,
             boxstyle="round,pad=0.02,rounding_size=0.08",
             linewidth=1.3, edgecolor='#7F8C8D', facecolor='#FCF8E3', zorder=4))
ax.text(notes_x + notes_w/2, notes_y + notes_h - 0.35, '◆ 焊接重點 ◆',
        ha='center', fontsize=10.5, fontweight='bold', color='#7D6608')

notes = [
    'S3 用 2×22 排母「可拔」',
    'NRF24 也用排母可拔',
    '100µF 焊 NRF24 VCC↔GND',
    'TFT 排線從頂部出',
    'B 排線從底部出',
    'buck 先量 5.0V 再接 S3',
    '虛線=跨側走線(走背面)',
    '只標 V2-A 用到的腳,',
    '其他保留給 V2-B(語音)',
]
for i, ln in enumerate(notes):
    ax.text(notes_x + 0.18, notes_y + notes_h - 0.85 - i * 0.28, '・' + ln,
            fontsize=8, color='#333', zorder=5)

plt.tight_layout()
out = 'handle_v2_a_board_layout.png'
plt.savefig(out, dpi=150, bbox_inches='tight', facecolor='white')
print(f'[OK] saved: {out}')
