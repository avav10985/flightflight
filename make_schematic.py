"""
無人機端飛控電路原理圖
仿 tx.JPG/rx.JPG 風格 - ESP32 + 模組 + 連接線
用 matplotlib 直接繪製，含實際接線
"""
import matplotlib.pyplot as plt
import matplotlib.patches as mp
from matplotlib.patches import FancyBboxPatch, Rectangle, Circle, Polygon
import matplotlib

matplotlib.rcParams['font.family'] = ['Microsoft JhengHei', 'sans-serif']
matplotlib.rcParams['axes.unicode_minus'] = False

fig, ax = plt.subplots(figsize=(18, 12), dpi=140)
ax.set_xlim(0, 24)
ax.set_ylim(0, 16)
ax.set_aspect('equal')
ax.axis('off')
fig.patch.set_facecolor('white')

# ----- 顏色 -----
BOX_EDGE = '#222'
BOX_FILL = '#FAFAFA'
WIRE = '#1A1A1A'
WIRE_PWR = '#C0392B'
WIRE_GND = '#000'
WIRE_I2C = '#2980B9'
WIRE_SPI = '#8E44AD'
WIRE_PWM = '#16A085'
TEXT = '#111'

# ----- 標題 -----
ax.text(12, 15.4, '四軸無人機 飛控電路原理圖', ha='center',
        fontsize=22, fontweight='bold', color=TEXT)
ax.text(12, 14.85, 'ESP32 + MPU6050 + NRF24L01+PA/LNA + ESC×4 + Battery Monitor',
        ha='center', fontsize=11, color='gray', style='italic')

# =================================================================
# 元件繪製函式
# =================================================================
def draw_ic(x, y, w, h, name, ref, pins_left, pins_right, fontsize=10):
    """畫一個 IC 矩形，含腳位標籤。pins_left/right = [pin_name, ...] 由上到下"""
    rect = Rectangle((x, y), w, h, linewidth=2, edgecolor=BOX_EDGE,
                     facecolor=BOX_FILL, zorder=3)
    ax.add_patch(rect)
    # 元件名稱（中間）
    ax.text(x + w/2, y + h/2 + 0.2, name, ha='center', va='center',
            fontsize=fontsize, fontweight='bold', zorder=4)
    # Reference designator (左上角)
    ax.text(x, y + h + 0.2, ref, ha='left', va='bottom',
            fontsize=10, fontweight='bold', color='#C0392B', zorder=4)
    # 左側腳位
    pin_positions_L = []
    if pins_left:
        n = len(pins_left)
        for i, pin in enumerate(pins_left):
            py = y + h * (n - i - 0.5) / n
            # 腳位短線
            ax.plot([x - 0.25, x], [py, py], color=BOX_EDGE, linewidth=1.5, zorder=4)
            # 腳位編號（IC 內側）
            ax.text(x + 0.1, py + 0.08, str(i+1), ha='left', va='bottom',
                    fontsize=7, color='#888', zorder=4)
            # 腳位名稱（IC 內側偏下）
            ax.text(x + 0.1, py - 0.08, pin, ha='left', va='top',
                    fontsize=9, color=TEXT, zorder=4)
            pin_positions_L.append((x - 0.25, py))
    # 右側腳位
    pin_positions_R = []
    if pins_right:
        n = len(pins_right)
        for i, pin in enumerate(pins_right):
            py = y + h * (n - i - 0.5) / n
            ax.plot([x + w, x + w + 0.25], [py, py], color=BOX_EDGE, linewidth=1.5, zorder=4)
            ax.text(x + w - 0.1, py + 0.08, str(len(pins_left) + i + 1), ha='right', va='bottom',
                    fontsize=7, color='#888', zorder=4)
            ax.text(x + w - 0.1, py - 0.08, pin, ha='right', va='top',
                    fontsize=9, color=TEXT, zorder=4)
            pin_positions_R.append((x + w + 0.25, py))
    return {'L': pin_positions_L, 'R': pin_positions_R, 'rect': (x, y, w, h)}


def wire(p1, p2, color=WIRE, lw=1.2, label='', mid=None, label_offset=(0, 0.15)):
    """畫一條 90° 折線 wire，可選 label。"""
    x1, y1 = p1
    x2, y2 = p2
    if mid is None:
        # 預設先水平再垂直
        ax.plot([x1, x2, x2], [y1, y1, y2], color=color, linewidth=lw, zorder=2)
        mx = (x1 + x2) / 2
        my = y1
    else:
        mx, my = mid
        ax.plot([x1, mx, mx, x2], [y1, y1, my, my], color=color, linewidth=lw, zorder=2)
        if y2 != my:
            ax.plot([x2, x2], [my, y2], color=color, linewidth=lw, zorder=2)
    # 連接點圓點
    ax.add_patch(Circle((x1, y1), 0.07, color=color, zorder=5))
    ax.add_patch(Circle((x2, y2), 0.07, color=color, zorder=5))
    if label:
        ax.text(mx + label_offset[0], my + label_offset[1], label,
                ha='center', va='bottom', fontsize=8, color=color,
                bbox=dict(boxstyle='round,pad=0.15', facecolor='white',
                          edgecolor='none', alpha=0.9), zorder=6)


def ground(x, y, label=''):
    """畫一個接地符號（向下）"""
    ax.plot([x, x], [y, y - 0.2], color=WIRE_GND, linewidth=1.5, zorder=4)
    ax.plot([x - 0.25, x + 0.25], [y - 0.2, y - 0.2], color=WIRE_GND, linewidth=2, zorder=4)
    ax.plot([x - 0.17, x + 0.17], [y - 0.3, y - 0.3], color=WIRE_GND, linewidth=1.5, zorder=4)
    ax.plot([x - 0.08, x + 0.08], [y - 0.4, y - 0.4], color=WIRE_GND, linewidth=1, zorder=4)
    if label:
        ax.text(x, y - 0.55, label, ha='center', va='top', fontsize=8, color=WIRE_GND)


def vdd(x, y, label='+VCC'):
    """畫一個 VCC 符號（向上）"""
    ax.plot([x, x], [y, y + 0.3], color=WIRE_PWR, linewidth=1.5, zorder=4)
    ax.plot([x - 0.18, x + 0.18], [y + 0.3, y + 0.3], color=WIRE_PWR, linewidth=2, zorder=4)
    ax.text(x, y + 0.45, label, ha='center', va='bottom', fontsize=8,
            color=WIRE_PWR, fontweight='bold')


def capacitor(x, y, label='', polar=True, orient='V'):
    """畫一個電容符號（簡化）"""
    if orient == 'V':
        ax.plot([x, x], [y, y - 0.1], color=BOX_EDGE, linewidth=1)
        ax.plot([x - 0.2, x + 0.2], [y - 0.1, y - 0.1], color=BOX_EDGE, linewidth=2)
        ax.plot([x - 0.2, x + 0.2], [y - 0.25, y - 0.25], color=BOX_EDGE, linewidth=2)
        ax.plot([x, x], [y - 0.25, y - 0.4], color=BOX_EDGE, linewidth=1)
        if polar:
            ax.text(x - 0.3, y - 0.05, '+', fontsize=10, color=BOX_EDGE)
        if label:
            ax.text(x + 0.35, y - 0.17, label, fontsize=8, va='center')


def resistor(x, y, length=1.0, label='', orient='V'):
    """畫一個電阻符號（鋸齒）"""
    if orient == 'V':
        zigzag_x = [x, x-0.15, x+0.15, x-0.15, x+0.15, x-0.15, x+0.15, x]
        zigzag_y = [y, y - 0.08, y - 0.16, y - 0.24, y - 0.32, y - 0.40, y - 0.48, y - length]
        ax.plot(zigzag_x, zigzag_y, color=BOX_EDGE, linewidth=1.3, zorder=3)
        if label:
            ax.text(x + 0.25, y - length/2, label, fontsize=8, va='center')

# =================================================================
# 畫 ESP32 主控板 (中央)
# =================================================================
esp32 = draw_ic(
    x=8.5, y=5, w=4, h=8,
    name='ESP32\nDevKit V1', ref='U1',
    pins_left=['GPIO16', 'GPIO17', 'GPIO22', 'GPIO21',
               'GPIO25', 'GPIO26', 'GPIO27', 'GPIO14',
               'GPIO34', 'VIN', '3V3', 'GND'],
    pins_right=['GND',  'GPIO5',  'GPIO4',
                'GPIO18','GPIO19','GPIO23',
                'TX0',  'RX0']
)
L = esp32['L']  # 左側腳位座標 list
R = esp32['R']  # 右側腳位座標 list

# 索引對應（從上往下）
P_GPIO16 = L[0];  P_GPIO17 = L[1]
P_GPIO22 = L[2];  P_GPIO21 = L[3]
P_GPIO25 = L[4];  P_GPIO26 = L[5];  P_GPIO27 = L[6];  P_GPIO14 = L[7]
P_GPIO34 = L[8]
P_VIN = L[9];    P_3V3 = L[10];    P_GND_L = L[11]

P_GND_R  = R[0];  P_GPIO5 = R[1];  P_GPIO4 = R[2]
P_GPIO18 = R[3];  P_GPIO19 = R[4]; P_GPIO23 = R[5]

# =================================================================
# MPU6050 (左側)
# =================================================================
mpu = draw_ic(
    x=2, y=9.5, w=3, h=3,
    name='MPU6050\n(I²C 0x68)', ref='U3',
    pins_left=[], pins_right=['SCL', 'SDA', 'GND', 'VCC']
)
MPU_SCL = mpu['R'][0]; MPU_SDA = mpu['R'][1]
MPU_GND = mpu['R'][2]; MPU_VCC = mpu['R'][3]

# I²C 連線
wire(MPU_SDA, P_GPIO21, color=WIRE_I2C, lw=1.5, label='SDA → GPIO21')
wire(MPU_SCL, P_GPIO22, color=WIRE_I2C, lw=1.5, label='SCL → GPIO22')

# MPU 電源
ax.plot([MPU_VCC[0], MPU_VCC[0] + 0.7], [MPU_VCC[1], MPU_VCC[1]], color=WIRE_PWR, linewidth=1.2)
vdd(MPU_VCC[0] + 0.7, MPU_VCC[1], '+3V3')
ax.plot([MPU_GND[0], MPU_GND[0] + 0.5], [MPU_GND[1], MPU_GND[1]], color=WIRE_GND, linewidth=1.2)
ground(MPU_GND[0] + 0.5, MPU_GND[1])

# =================================================================
# NRF24L01+PA/LNA (右側)
# =================================================================
nrf = draw_ic(
    x=16, y=8, w=3.5, h=4.5,
    name='NRF24L01\n+PA/LNA', ref='U2',
    pins_left=['VCC', 'GND', 'CE', 'CSN'],
    pins_right=['SCK', 'MOSI', 'MISO', 'IRQ']
)
NRF_VCC = nrf['L'][0]; NRF_GND = nrf['L'][1]
NRF_CE  = nrf['L'][2]; NRF_CSN = nrf['L'][3]
NRF_SCK = nrf['R'][0]; NRF_MOSI = nrf['R'][1]; NRF_MISO = nrf['R'][2]

# SPI 連線 (CE/CSN 直接連到 ESP32 右側對應 pin)
wire(NRF_CE, P_GPIO4, color=WIRE_SPI, lw=1.5, label='CE → GPIO4')
wire(NRF_CSN, P_GPIO5, color=WIRE_SPI, lw=1.5, label='CSN → GPIO5')
# SCK/MOSI/MISO 從 NRF 右側拉到上方再繞回 ESP32 右側
wire(NRF_SCK, P_GPIO18, color=WIRE_SPI, lw=1.5, label='SCK → GPIO18', mid=(20.5, 13))
wire(NRF_MOSI, P_GPIO23, color=WIRE_SPI, lw=1.5, label='MOSI → GPIO23', mid=(20.8, 13.4))
wire(NRF_MISO, P_GPIO19, color=WIRE_SPI, lw=1.5, label='MISO → GPIO19', mid=(21.1, 13.8))

# NRF24 電源（含 100μF 去耦電容）
ax.plot([NRF_VCC[0] - 0.5, NRF_VCC[0]], [NRF_VCC[1], NRF_VCC[1]], color=WIRE_PWR, linewidth=1.2)
capacitor(NRF_VCC[0] - 0.5, NRF_VCC[1] + 0.5, label='C1 100µF')
ax.plot([NRF_VCC[0] - 0.5, NRF_VCC[0] - 0.5], [NRF_VCC[1] + 0.5, NRF_VCC[1] + 1.0], color=WIRE_PWR, linewidth=1.2)
vdd(NRF_VCC[0] - 0.5, NRF_VCC[1] + 1.0, '+3V3')
# C1 接地端
ax.plot([NRF_VCC[0] - 0.5, NRF_VCC[0] - 0.5], [NRF_VCC[1] + 0.1, NRF_VCC[1]], color=WIRE_GND, linewidth=1)
ax.plot([NRF_VCC[0] - 0.5, NRF_VCC[0] - 1.0], [NRF_VCC[1], NRF_VCC[1]], color=WIRE_GND, linewidth=1)
ground(NRF_VCC[0] - 1.0, NRF_VCC[1])

ax.plot([NRF_GND[0] - 0.5, NRF_GND[0]], [NRF_GND[1], NRF_GND[1]], color=WIRE_GND, linewidth=1.2)
ground(NRF_GND[0] - 0.5, NRF_GND[1])

# =================================================================
# ESC × 4 (底部)
# =================================================================
esc_data = [
    (1.5, 2, 'ESC1', 'M1 前左 CW',  P_GPIO25, 'GPIO25'),
    (6,   2, 'ESC2', 'M2 前右 CCW', P_GPIO26, 'GPIO26'),
    (14,  2, 'ESC3', 'M3 後左 CCW', P_GPIO27, 'GPIO27'),
    (18.5,2, 'ESC4', 'M4 後右 CW',  P_GPIO14, 'GPIO14'),
]
for x, y, ref, motor, esp_pin, gpio_name in esc_data:
    esc = draw_ic(
        x=x, y=y, w=3, h=1.5,
        name=f'{ref}\n{motor}', ref=ref,
        pins_left=[], pins_right=[]
    )
    # 三條訊號腳出去
    pin_y = y + 1.5
    for i, (label_name, dy) in enumerate([('SIG', 0.4), ('+', 1.0), ('GND', 1.6)]):
        px = x + 0.4 + i * 1.05
        ax.plot([px, px], [pin_y, pin_y + 0.2], color=BOX_EDGE, linewidth=1.5)
        ax.add_patch(Circle((px, pin_y + 0.2), 0.06, color=BOX_EDGE, zorder=5))
        ax.text(px, pin_y + 0.3, label_name, ha='center', va='bottom',
                fontsize=8, color=TEXT)
    # SIG 線連到 ESP32
    sig_x = x + 0.4
    wire((sig_x, pin_y + 0.2), esp_pin, color=WIRE_PWM, lw=1.5, label=f'SIG → {gpio_name}')
    # + 接 +11.1V (動力)
    pwr_x = x + 0.4 + 1.05
    ax.plot([pwr_x, pwr_x], [pin_y + 0.2, pin_y + 0.6], color=WIRE_PWR, linewidth=1.5)
    vdd(pwr_x, pin_y + 0.6, '+11.1V')
    # GND
    gnd_x = x + 0.4 + 2.1
    ax.plot([gnd_x, gnd_x], [pin_y + 0.2, pin_y + 0.5], color=WIRE_GND, linewidth=1.2)
    ground(gnd_x, pin_y + 0.5)

# =================================================================
# 電池監測分壓 (右下角)
# =================================================================
batt_x = 22; batt_y = 7
ax.text(batt_x, batt_y + 0.3, 'BAT+', ha='center', fontsize=10, fontweight='bold',
        color=WIRE_PWR)
ax.plot([batt_x, batt_x], [batt_y + 0.2, batt_y], color=WIRE_PWR, linewidth=1.5)
resistor(batt_x, batt_y, length=1.0, label='R1 30kΩ')
# 中點
mid_y = batt_y - 1.0
ax.add_patch(Circle((batt_x, mid_y), 0.08, color=BOX_EDGE, zorder=5))
resistor(batt_x, mid_y, length=1.0, label='R2 10kΩ')
ax.plot([batt_x, batt_x], [mid_y - 1.0, mid_y - 1.3], color=WIRE_GND, linewidth=1.2)
ground(batt_x, mid_y - 1.3)
# 中點接 GPIO34
wire((batt_x, mid_y), P_GPIO34, color='#27AE60', lw=1.5, label='中點 → GPIO34 (ADC)')

# =================================================================
# ESP32 電源
# =================================================================
ax.plot([P_3V3[0] - 0.5, P_3V3[0]], [P_3V3[1], P_3V3[1]], color=WIRE_PWR, linewidth=1.2)
vdd(P_3V3[0] - 0.5, P_3V3[1], '+3V3')
ax.plot([P_VIN[0] - 0.5, P_VIN[0]], [P_VIN[1], P_VIN[1]], color=WIRE_PWR, linewidth=1.2)
vdd(P_VIN[0] - 0.5, P_VIN[1], '+5V')
ax.plot([P_GND_L[0] - 0.5, P_GND_L[0]], [P_GND_L[1], P_GND_L[1]], color=WIRE_GND, linewidth=1.2)
ground(P_GND_L[0] - 0.5, P_GND_L[1])
ax.plot([P_GND_R[0], P_GND_R[0] + 0.5], [P_GND_R[1], P_GND_R[1]], color=WIRE_GND, linewidth=1.2)
ground(P_GND_R[0] + 0.5, P_GND_R[1])

# =================================================================
# 預留腳位（GPS 用，未連接）
# =================================================================
ax.plot([P_GPIO16[0], P_GPIO16[0] - 0.8], [P_GPIO16[1], P_GPIO16[1]],
        color='gray', linewidth=1, linestyle='--')
ax.text(P_GPIO16[0] - 1, P_GPIO16[1], '→ GPS TX (預留)', ha='right',
        va='center', fontsize=8, color='gray', style='italic')
ax.plot([P_GPIO17[0], P_GPIO17[0] - 0.8], [P_GPIO17[1], P_GPIO17[1]],
        color='gray', linewidth=1, linestyle='--')
ax.text(P_GPIO17[0] - 1, P_GPIO17[1], '→ GPS RX (預留)', ha='right',
        va='center', fontsize=8, color='gray', style='italic')

# =================================================================
# 線色圖例 (左上)
# =================================================================
legend_x, legend_y = 0.5, 13.5
ax.add_patch(Rectangle((legend_x - 0.2, legend_y - 1.8), 4.2, 2.0,
                       linewidth=1, edgecolor='gray', facecolor='#F5F5F5', alpha=0.85, zorder=1))
ax.text(legend_x, legend_y, '線色說明：', fontsize=9, fontweight='bold', zorder=2)
legend_items = [
    (WIRE_I2C, 'I²C (SDA/SCL)'),
    (WIRE_SPI, 'SPI (NRF24)'),
    (WIRE_PWM, 'PWM 訊號 (ESC)'),
    (WIRE_PWR, '電源 +VCC'),
    (WIRE_GND, 'GND'),
]
for i, (color, lbl) in enumerate(legend_items):
    ax.plot([legend_x, legend_x + 0.5], [legend_y - 0.25 - i*0.25] * 2,
            color=color, linewidth=2, zorder=2)
    ax.text(legend_x + 0.6, legend_y - 0.25 - i*0.25, lbl,
            fontsize=8, va='center', zorder=2)

# =================================================================
# 標題欄 (右下)
# =================================================================
tbox_x, tbox_y = 17.5, 0.3
ax.add_patch(Rectangle((tbox_x, tbox_y), 6.2, 1.4,
                       linewidth=1.5, edgecolor=BOX_EDGE, facecolor='white', zorder=3))
ax.plot([tbox_x, tbox_x + 6.2], [tbox_y + 0.7, tbox_y + 0.7],
        color=BOX_EDGE, linewidth=1, zorder=4)
ax.plot([tbox_x + 2, tbox_x + 2], [tbox_y, tbox_y + 1.4],
        color=BOX_EDGE, linewidth=1, zorder=4)
ax.text(tbox_x + 1, tbox_y + 1.05, '標題', ha='center', fontsize=8, color='gray')
ax.text(tbox_x + 1, tbox_y + 0.35, '日期', ha='center', fontsize=8, color='gray')
ax.text(tbox_x + 4.1, tbox_y + 1.05, '飛控電路圖', ha='center', fontsize=9, fontweight='bold')
ax.text(tbox_x + 4.1, tbox_y + 0.35, '2026-05', ha='center', fontsize=9)

plt.savefig('d:/無人機/flightflight/drone_schematic.png',
            dpi=140, bbox_inches='tight', facecolor='white')
print("Saved: drone_schematic.png")
