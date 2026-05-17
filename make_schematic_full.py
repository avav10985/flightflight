"""
完整版無人機端電路圖（全部模組到位）
- ESP32 + MPU6050 + BMP280 + NEO-6M GPS
- TCA9548A + VL53L1X × 2（之後加到 4）
- NRF24L01+PA/LNA + 100µF 電容
- ESC × 4
- 電池分壓監測
- ESP32-CAM（獨立電源）
- 右側附「接線對照表」
"""
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle, FancyBboxPatch
import matplotlib

matplotlib.rcParams['font.family'] = ['Microsoft JhengHei', 'sans-serif']
matplotlib.rcParams['axes.unicode_minus'] = False

fig, ax = plt.subplots(figsize=(22, 14), dpi=120)
ax.set_xlim(0, 28)
ax.set_ylim(0, 18)
ax.set_aspect('equal')
ax.axis('off')
fig.patch.set_facecolor('white')

# ===== 顏色 =====
BOX_EDGE = '#222'
BOX_FILL = '#FAFAFA'
TEXT = '#111'
WIRE_I2C = '#2980B9'
WIRE_SPI = '#8E44AD'
WIRE_UART = '#D35400'
WIRE_PWM = '#16A085'
WIRE_PWR = '#C0392B'
WIRE_GND = '#000'
WIRE_ADC = '#27AE60'

# ===== 標題 =====
ax.text(14, 17.5, '四軸無人機 完整電路圖（飛控全配）', ha='center',
        fontsize=22, fontweight='bold', color=TEXT)
ax.text(14, 17.0,
        'ESP32 + MPU6050 + BMP280 + NEO-6M + TCA9548A + VL53L1X×2 + NRF24L01+PA/LNA + ESC×4 + ESP32-CAM',
        ha='center', fontsize=10, color='gray', style='italic')

# =================================================================
# 元件繪製
# =================================================================
def draw_ic(x, y, w, h, name, ref, pins_left=None, pins_right=None, pins_bot=None, fill=BOX_FILL):
    rect = Rectangle((x, y), w, h, linewidth=2, edgecolor=BOX_EDGE,
                     facecolor=fill, zorder=3)
    ax.add_patch(rect)
    ax.text(x + w/2, y + h/2 + 0.15, name, ha='center', va='center',
            fontsize=10, fontweight='bold', zorder=4)
    ax.text(x, y + h + 0.18, ref, ha='left', va='bottom',
            fontsize=10, fontweight='bold', color='#C0392B', zorder=4)
    out = {'L': [], 'R': [], 'B': [], 'rect': (x, y, w, h)}
    if pins_left:
        n = len(pins_left)
        for i, pin in enumerate(pins_left):
            py = y + h * (n - i - 0.5) / n
            ax.plot([x - 0.3, x], [py, py], color=BOX_EDGE, linewidth=1.5, zorder=4)
            ax.text(x + 0.1, py, pin, ha='left', va='center',
                    fontsize=8.5, color=TEXT, zorder=4)
            out['L'].append((x - 0.3, py))
    if pins_right:
        n = len(pins_right)
        for i, pin in enumerate(pins_right):
            py = y + h * (n - i - 0.5) / n
            ax.plot([x + w, x + w + 0.3], [py, py], color=BOX_EDGE, linewidth=1.5, zorder=4)
            ax.text(x + w - 0.1, py, pin, ha='right', va='center',
                    fontsize=8.5, color=TEXT, zorder=4)
            out['R'].append((x + w + 0.3, py))
    if pins_bot:
        n = len(pins_bot)
        for i, pin in enumerate(pins_bot):
            px = x + w * (i + 0.5) / n
            ax.plot([px, px], [y, y - 0.3], color=BOX_EDGE, linewidth=1.5, zorder=4)
            ax.text(px, y + 0.15, pin, ha='center', va='bottom',
                    fontsize=8.5, color=TEXT, zorder=4)
            out['B'].append((px, y - 0.3))
    return out


def wire(p1, p2, color, lw=1.4, label=None, label_pos=0.5, mid=None):
    """畫連接線。p1, p2 為座標 tuple。mid 指定轉折點。"""
    x1, y1 = p1
    x2, y2 = p2
    if mid is None:
        if abs(y2 - y1) < 0.1:
            ax.plot([x1, x2], [y1, y2], color=color, linewidth=lw, zorder=2)
            mid_xy = ((x1 + x2) / 2, y1)
        else:
            # 預設：先水平再垂直
            ax.plot([x1, x2, x2], [y1, y1, y2], color=color, linewidth=lw, zorder=2)
            mid_xy = (x2, (y1 + y2) / 2)
    else:
        mx, my = mid
        ax.plot([x1, mx, mx, x2, x2], [y1, y1, my, my, y2],
                color=color, linewidth=lw, zorder=2)
        mid_xy = (mx, my)
    ax.add_patch(Circle((x1, y1), 0.06, color=color, zorder=5))
    ax.add_patch(Circle((x2, y2), 0.06, color=color, zorder=5))
    if label:
        ax.text(mid_xy[0], mid_xy[1] + 0.18, label,
                ha='center', va='bottom', fontsize=7.5, color=color,
                bbox=dict(boxstyle='round,pad=0.15', facecolor='white',
                          edgecolor=color, alpha=0.95, linewidth=0.5),
                zorder=6)


def gnd_symbol(x, y, size=0.25):
    ax.plot([x, x], [y, y - size * 0.6], color=WIRE_GND, linewidth=1.5, zorder=4)
    ax.plot([x - size, x + size], [y - size * 0.6, y - size * 0.6], color=WIRE_GND, linewidth=2, zorder=4)
    ax.plot([x - size * 0.7, x + size * 0.7], [y - size * 0.9, y - size * 0.9], color=WIRE_GND, linewidth=1.5, zorder=4)
    ax.plot([x - size * 0.35, x + size * 0.35], [y - size * 1.2, y - size * 1.2], color=WIRE_GND, linewidth=1, zorder=4)


def vdd_symbol(x, y, label='+VCC'):
    ax.plot([x, x], [y, y + 0.3], color=WIRE_PWR, linewidth=1.5, zorder=4)
    ax.plot([x - 0.2, x + 0.2], [y + 0.3, y + 0.3], color=WIRE_PWR, linewidth=2.2, zorder=4)
    ax.text(x, y + 0.4, label, ha='center', va='bottom', fontsize=9,
            color=WIRE_PWR, fontweight='bold')


def cap_symbol(x, y, label=''):
    ax.plot([x, x], [y + 0.4, y + 0.2], color=BOX_EDGE, linewidth=1.5)
    ax.plot([x - 0.2, x + 0.2], [y + 0.2, y + 0.2], color=BOX_EDGE, linewidth=2.2)
    ax.plot([x - 0.2, x + 0.2], [y + 0.05, y + 0.05], color=BOX_EDGE, linewidth=2.2)
    ax.plot([x, x], [y + 0.05, y - 0.1], color=BOX_EDGE, linewidth=1.5)
    ax.text(x - 0.3, y + 0.35, '+', fontsize=10, color=BOX_EDGE)
    ax.text(x + 0.3, y + 0.13, label, fontsize=8, va='center')


def resistor_v(x, y, length, label=''):
    """垂直電阻"""
    zx = [x, x-0.13, x+0.13, x-0.13, x+0.13, x-0.13, x+0.13, x]
    zy = [y, y-length*0.12, y-length*0.24, y-length*0.36,
          y-length*0.48, y-length*0.6, y-length*0.72, y-length]
    ax.plot(zx, zy, color=BOX_EDGE, linewidth=1.4, zorder=3)
    ax.text(x + 0.3, y - length/2, label, fontsize=8.5, va='center')


# =================================================================
# ESP32 主控（中央偏左）
# =================================================================
esp = draw_ic(
    x=9, y=4.5, w=4.5, h=10,
    name='ESP32\nDevKit V1', ref='U1',
    pins_left=['GPIO16 (RX2)', 'GPIO17 (TX2)',
               'GPIO22 (SCL)', 'GPIO21 (SDA)',
               'GPIO25', 'GPIO26', 'GPIO27', 'GPIO14',
               'GPIO34 (ADC)',
               'VIN (5V)', '3V3', 'GND'],
    pins_right=['GND',
                'GPIO5 (CSN)', 'GPIO4 (CE)',
                'GPIO18 (SCK)', 'GPIO19 (MISO)', 'GPIO23 (MOSI)'],
    fill='#EBEDEF'
)
L = esp['L']; R = esp['R']
P_G16  = L[0];  P_G17  = L[1]
P_G22  = L[2];  P_G21  = L[3]
P_G25  = L[4];  P_G26  = L[5];  P_G27  = L[6];  P_G14  = L[7]
P_G34  = L[8]
P_VIN  = L[9];  P_3V3  = L[10]; P_GNDL = L[11]
P_GNDR = R[0]
P_G5   = R[1];  P_G4   = R[2]
P_G18  = R[3];  P_G19  = R[4];  P_G23  = R[5]

# =================================================================
# I²C 匯流排幹線（左側上下走線）
# =================================================================
I2C_BUS_X = 5.5
# 拉一條 I²C 主幹線
ax.plot([I2C_BUS_X, I2C_BUS_X], [6.5, 14.5], color=WIRE_I2C, linewidth=2.5, zorder=2)
ax.plot([I2C_BUS_X + 0.4, I2C_BUS_X + 0.4], [6.5, 14.5], color=WIRE_I2C, linewidth=2.5, zorder=2)
ax.text(I2C_BUS_X - 0.7, 14.7, 'SDA', fontsize=9, color=WIRE_I2C, fontweight='bold')
ax.text(I2C_BUS_X + 0.5, 14.7, 'SCL', fontsize=9, color=WIRE_I2C, fontweight='bold')

# I²C 主幹接到 ESP32
wire((I2C_BUS_X, P_G21[1]), P_G21, color=WIRE_I2C, lw=1.8)
wire((I2C_BUS_X + 0.4, P_G22[1]), P_G22, color=WIRE_I2C, lw=1.8)
ax.text((I2C_BUS_X + P_G21[0])/2, P_G21[1] - 0.25, 'SDA → GPIO21',
        ha='center', fontsize=7.5, color=WIRE_I2C,
        bbox=dict(boxstyle='round,pad=0.1', facecolor='white', edgecolor=WIRE_I2C, linewidth=0.5))
ax.text((I2C_BUS_X + P_G22[0])/2, P_G22[1] + 0.15, 'SCL → GPIO22',
        ha='center', fontsize=7.5, color=WIRE_I2C,
        bbox=dict(boxstyle='round,pad=0.1', facecolor='white', edgecolor=WIRE_I2C, linewidth=0.5))

# =================================================================
# MPU6050（左上）
# =================================================================
mpu = draw_ic(
    x=1, y=12.5, w=3.5, h=2, name='MPU6050\n(I²C 0x68)', ref='U2',
    pins_right=['VCC', 'GND', 'SDA', 'SCL']
)
MPU_VCC, MPU_GND, MPU_SDA, MPU_SCL = mpu['R']
# I²C 接到主幹
ax.plot([MPU_SDA[0], I2C_BUS_X], [MPU_SDA[1], MPU_SDA[1]], color=WIRE_I2C, linewidth=1.5)
ax.add_patch(Circle((I2C_BUS_X, MPU_SDA[1]), 0.1, color=WIRE_I2C, zorder=5))
ax.plot([MPU_SCL[0], I2C_BUS_X + 0.4], [MPU_SCL[1], MPU_SCL[1]], color=WIRE_I2C, linewidth=1.5)
ax.add_patch(Circle((I2C_BUS_X + 0.4, MPU_SCL[1]), 0.1, color=WIRE_I2C, zorder=5))
# 電源
ax.plot([MPU_VCC[0], MPU_VCC[0] + 0.4], [MPU_VCC[1]] * 2, color=WIRE_PWR, linewidth=1.5)
vdd_symbol(MPU_VCC[0] + 0.4, MPU_VCC[1], '+3V3')
ax.plot([MPU_GND[0], MPU_GND[0] + 0.4], [MPU_GND[1]] * 2, color=WIRE_GND, linewidth=1.2)
gnd_symbol(MPU_GND[0] + 0.4, MPU_GND[1])

# =================================================================
# BMP280（左中）
# =================================================================
bmp = draw_ic(
    x=1, y=10, w=3.5, h=2, name='BMP280\n(I²C 0x76)', ref='U3',
    pins_right=['VCC', 'GND', 'SDA', 'SCL']
)
BMP_VCC, BMP_GND, BMP_SDA, BMP_SCL = bmp['R']
ax.plot([BMP_SDA[0], I2C_BUS_X], [BMP_SDA[1], BMP_SDA[1]], color=WIRE_I2C, linewidth=1.5)
ax.add_patch(Circle((I2C_BUS_X, BMP_SDA[1]), 0.1, color=WIRE_I2C, zorder=5))
ax.plot([BMP_SCL[0], I2C_BUS_X + 0.4], [BMP_SCL[1], BMP_SCL[1]], color=WIRE_I2C, linewidth=1.5)
ax.add_patch(Circle((I2C_BUS_X + 0.4, BMP_SCL[1]), 0.1, color=WIRE_I2C, zorder=5))
ax.plot([BMP_VCC[0], BMP_VCC[0] + 0.4], [BMP_VCC[1]] * 2, color=WIRE_PWR, linewidth=1.5)
vdd_symbol(BMP_VCC[0] + 0.4, BMP_VCC[1], '+3V3')
ax.plot([BMP_GND[0], BMP_GND[0] + 0.4], [BMP_GND[1]] * 2, color=WIRE_GND, linewidth=1.2)
gnd_symbol(BMP_GND[0] + 0.4, BMP_GND[1])

# =================================================================
# TCA9548A I²C 多工器（左下）→ VL53L1X
# =================================================================
tca = draw_ic(
    x=1, y=6.5, w=3.5, h=3, name='TCA9548A\n(I²C 0x70)', ref='U4',
    pins_right=['VCC', 'GND', 'SDA', 'SCL', 'SD0/SC0', 'SD1/SC1']
)
TCA_VCC, TCA_GND, TCA_SDA, TCA_SCL, TCA_CH0, TCA_CH1 = tca['R']
ax.plot([TCA_SDA[0], I2C_BUS_X], [TCA_SDA[1], TCA_SDA[1]], color=WIRE_I2C, linewidth=1.5)
ax.add_patch(Circle((I2C_BUS_X, TCA_SDA[1]), 0.1, color=WIRE_I2C, zorder=5))
ax.plot([TCA_SCL[0], I2C_BUS_X + 0.4], [TCA_SCL[1], TCA_SCL[1]], color=WIRE_I2C, linewidth=1.5)
ax.add_patch(Circle((I2C_BUS_X + 0.4, TCA_SCL[1]), 0.1, color=WIRE_I2C, zorder=5))
ax.plot([TCA_VCC[0], TCA_VCC[0] + 0.4], [TCA_VCC[1]] * 2, color=WIRE_PWR, linewidth=1.5)
vdd_symbol(TCA_VCC[0] + 0.4, TCA_VCC[1], '+3V3')
ax.plot([TCA_GND[0], TCA_GND[0] + 0.4], [TCA_GND[1]] * 2, color=WIRE_GND, linewidth=1.2)
gnd_symbol(TCA_GND[0] + 0.4, TCA_GND[1])

# VL53L1X ×2
vl1 = draw_ic(x=0.3, y=4.7, w=2.1, h=1.5, name='VL53L1X #1\n(前)', ref='U5',
              pins_right=['SCL', 'SDA'])
vl2 = draw_ic(x=2.5, y=4.7, w=2.1, h=1.5, name='VL53L1X #2\n(後)', ref='U6',
              pins_right=['SCL', 'SDA'])
# 連到 TCA 通道
ax.plot([vl1['R'][1][0], TCA_CH0[0]], [vl1['R'][1][1], TCA_CH0[1]], color=WIRE_I2C, linewidth=1.3, linestyle='-.')
ax.plot([vl1['R'][0][0], TCA_CH0[0] - 0.1], [vl1['R'][0][1], TCA_CH0[1]], color=WIRE_I2C, linewidth=1.3, linestyle='-.')
ax.text(2.3, 5.6, '#1 SDA/SCL\n→ CH0', fontsize=7, color=WIRE_I2C,
        bbox=dict(boxstyle='round,pad=0.1', facecolor='white', edgecolor=WIRE_I2C, linewidth=0.5))
ax.text(4.8, 5.6, '#2 SDA/SCL\n→ CH1', fontsize=7, color=WIRE_I2C,
        bbox=dict(boxstyle='round,pad=0.1', facecolor='white', edgecolor=WIRE_I2C, linewidth=0.5))
ax.plot([vl2['R'][1][0], TCA_CH1[0]], [vl2['R'][1][1], TCA_CH1[1]], color=WIRE_I2C, linewidth=1.3, linestyle='-.')

# VL53L1X 電源（從 TCA 拉 3V3 過去，簡化標註）
ax.text(0.3, 4.4, 'VCC→3V3、GND→GND（共匯）', fontsize=7, color='gray', style='italic')

# =================================================================
# NRF24L01+PA/LNA（右上）
# =================================================================
nrf = draw_ic(
    x=17, y=9.5, w=4, h=4.5, name='NRF24L01\n+PA/LNA', ref='U7',
    pins_left=['VCC', 'GND', 'CE', 'CSN'],
    pins_right=['SCK', 'MOSI', 'MISO', 'IRQ']
)
NRF_VCC, NRF_GND, NRF_CE, NRF_CSN = nrf['L']
NRF_SCK, NRF_MOSI, NRF_MISO, NRF_IRQ = nrf['R']

# CE/CSN 直接連 ESP32 右側
wire(NRF_CE, P_G4, color=WIRE_SPI, lw=1.6, label='CE → GPIO4')
wire(NRF_CSN, P_G5, color=WIRE_SPI, lw=1.6, label='CSN → GPIO5')
# SCK/MOSI/MISO 從右繞回（用 mid 點轉折）
wire(NRF_SCK, P_G18, color=WIRE_SPI, lw=1.6, label='SCK → GPIO18', mid=(22.5, 13.6))
wire(NRF_MOSI, P_G23, color=WIRE_SPI, lw=1.6, label='MOSI → GPIO23', mid=(22.8, 14.0))
wire(NRF_MISO, P_G19, color=WIRE_SPI, lw=1.6, label='MISO → GPIO19', mid=(23.1, 14.4))

# NRF24 電源 + 去耦電容 C1 100µF
ax.plot([NRF_VCC[0] - 0.5, NRF_VCC[0]], [NRF_VCC[1]] * 2, color=WIRE_PWR, linewidth=1.5)
cap_symbol(NRF_VCC[0] - 0.5, NRF_VCC[1] - 0.3, label='C1\n100µF')
# C1 的 + 接到 VCC、− 接到 GND
ax.plot([NRF_VCC[0] - 0.5] * 2, [NRF_VCC[1] + 0.4, NRF_VCC[1] + 0.8], color=WIRE_PWR, linewidth=1.5)
vdd_symbol(NRF_VCC[0] - 0.5, NRF_VCC[1] + 0.8, '+3V3')
ax.plot([NRF_VCC[0] - 0.5, NRF_VCC[0] - 1.0], [NRF_VCC[1] - 0.4, NRF_VCC[1] - 0.4], color=WIRE_GND, linewidth=1.2)
gnd_symbol(NRF_VCC[0] - 1.0, NRF_VCC[1] - 0.4)
ax.plot([NRF_GND[0], NRF_GND[0] - 0.5], [NRF_GND[1]] * 2, color=WIRE_GND, linewidth=1.2)
gnd_symbol(NRF_GND[0] - 0.5, NRF_GND[1])

# =================================================================
# NEO-6M GPS（右中）
# =================================================================
gps = draw_ic(
    x=17, y=6, w=4, h=2.5, name='GY-NEO6MV2\nGPS', ref='U8',
    pins_left=['VCC', 'GND', 'TX', 'RX']
)
GPS_VCC, GPS_GND, GPS_TX, GPS_RX = gps['L']
# TX → ESP32 GPIO16 (RX2)
wire(GPS_TX, P_G16, color=WIRE_UART, lw=1.6, label='GPS TX → GPIO16 (RX2)', mid=(15.5, 7.5))
# RX → ESP32 GPIO17 (TX2)
wire(GPS_RX, P_G17, color=WIRE_UART, lw=1.6, label='GPS RX → GPIO17 (TX2)', mid=(15.7, 6.8))
# 電源 (GPS 用 5V 較穩)
ax.plot([GPS_VCC[0] - 0.5, GPS_VCC[0]], [GPS_VCC[1]] * 2, color=WIRE_PWR, linewidth=1.5)
vdd_symbol(GPS_VCC[0] - 0.5, GPS_VCC[1], '+5V')
ax.plot([GPS_GND[0] - 0.5, GPS_GND[0]], [GPS_GND[1]] * 2, color=WIRE_GND, linewidth=1.2)
gnd_symbol(GPS_GND[0] - 0.5, GPS_GND[1])

# =================================================================
# ESC × 4（底部）
# =================================================================
esc_x_positions = [9, 11.7, 14.4, 17.1]
esc_pins_to_esp = [(P_G25, 'M1 前左 CW'),
                   (P_G26, 'M2 前右 CCW'),
                   (P_G27, 'M3 後左 CCW'),
                   (P_G14, 'M4 後右 CW')]
esc_gpio_names = ['GPIO25', 'GPIO26', 'GPIO27', 'GPIO14']

for i, (x, (esp_pin, motor), gpio) in enumerate(zip(esc_x_positions, esc_pins_to_esp, esc_gpio_names)):
    esc = draw_ic(
        x=x, y=1, w=2.3, h=1.4,
        name=f'ESC{i+1}\n{motor}', ref=f'ESC{i+1}',
        pins_bot=['SIG', '+', 'GND']
    )
    SIG, PWR, GND = esc['B']
    # SIG → ESP32 GPIO
    wire((SIG[0], 1), esp_pin, color=WIRE_PWM, lw=1.6,
         label=f'SIG → {gpio}', mid=(SIG[0], 3.5))
    # 動力 +11.1V
    ax.plot([PWR[0], PWR[0]], [1, 0.4], color=WIRE_PWR, linewidth=1.5)
    vdd_symbol(PWR[0], 0.4, '+11.1V')
    # GND
    ax.plot([GND[0], GND[0]], [1, 0.4], color=WIRE_GND, linewidth=1.2)
    gnd_symbol(GND[0], 0.4)

# =================================================================
# 電池監測（右下）
# =================================================================
batt_x = 24
ax.text(batt_x, 5.6, 'BAT+', ha='center', fontsize=11, fontweight='bold', color=WIRE_PWR)
ax.plot([batt_x, batt_x], [5.5, 5.3], color=WIRE_PWR, linewidth=1.5)
resistor_v(batt_x, 5.3, 1.2, label='R1 30kΩ')
ax.add_patch(Circle((batt_x, 4.1), 0.1, color=BOX_EDGE, zorder=5))
resistor_v(batt_x, 4.1, 1.2, label='R2 10kΩ')
ax.plot([batt_x, batt_x], [2.9, 2.6], color=WIRE_GND, linewidth=1.2)
gnd_symbol(batt_x, 2.6)
# 中點 → GPIO34
wire((batt_x, 4.1), P_G34, color=WIRE_ADC, lw=1.6,
     label='中點 → GPIO34 (ADC)', mid=(22, 4.1))

# =================================================================
# ESP32-CAM（獨立島）
# =================================================================
cam = draw_ic(
    x=22.5, y=8.5, w=4.5, h=3.5,
    name='ESP32-CAM\n(AI-Thinker)', ref='U9',
    pins_left=['VCC (5V)', 'GND'],
    fill='#FFF3CD'
)
CAM_VCC, CAM_GND = cam['L']
ax.plot([CAM_VCC[0] - 0.5, CAM_VCC[0]], [CAM_VCC[1]] * 2, color=WIRE_PWR, linewidth=1.5)
vdd_symbol(CAM_VCC[0] - 0.5, CAM_VCC[1], '+5V')
ax.plot([CAM_GND[0] - 0.5, CAM_GND[0]], [CAM_GND[1]] * 2, color=WIRE_GND, linewidth=1.2)
gnd_symbol(CAM_GND[0] - 0.5, CAM_GND[1])

# ESP32-CAM 內含 WiFi 圖示
ax.annotate('(((  WiFi → 電腦',
            xy=(cam['rect'][0] + cam['rect'][2] - 0.3, cam['rect'][1] + cam['rect'][3] - 0.5),
            ha='right', fontsize=9, color='gray', fontweight='bold')
ax.text(cam['rect'][0] + 0.2, cam['rect'][1] + 0.3,
        '⚠ 不接 ESP32 訊號\n只共用電池電源',
        fontsize=7.5, color='gray', style='italic')

# =================================================================
# ESP32 電源輸入
# =================================================================
ax.plot([P_VIN[0] - 0.5, P_VIN[0]], [P_VIN[1]] * 2, color=WIRE_PWR, linewidth=1.5)
vdd_symbol(P_VIN[0] - 0.5, P_VIN[1], '+5V (BEC)')
ax.plot([P_3V3[0] - 0.5, P_3V3[0]], [P_3V3[1]] * 2, color=WIRE_PWR, linewidth=1.5)
vdd_symbol(P_3V3[0] - 0.5, P_3V3[1], '+3V3 (內穩壓)')
ax.plot([P_GNDL[0] - 0.5, P_GNDL[0]], [P_GNDL[1]] * 2, color=WIRE_GND, linewidth=1.2)
gnd_symbol(P_GNDL[0] - 0.5, P_GNDL[1])
ax.plot([P_GNDR[0], P_GNDR[0] + 0.5], [P_GNDR[1]] * 2, color=WIRE_GND, linewidth=1.2)
gnd_symbol(P_GNDR[0] + 0.5, P_GNDR[1])

# =================================================================
# 圖例（左上角）
# =================================================================
ax.add_patch(Rectangle((0.3, 15.0), 3.8, 2.0, linewidth=1.2,
                       edgecolor='gray', facecolor='#F8F9FA', alpha=0.95, zorder=1))
ax.text(0.5, 16.8, '線色說明', fontsize=10, fontweight='bold', zorder=2)
legend = [
    (WIRE_I2C, 'I²C (SDA/SCL)'),
    (WIRE_SPI, 'SPI (NRF24)'),
    (WIRE_UART, 'UART (GPS)'),
    (WIRE_PWM, 'PWM (ESC)'),
    (WIRE_PWR, '電源 VCC'),
    (WIRE_GND, 'GND'),
    (WIRE_ADC, 'ADC (電池)'),
]
for i, (c, lbl) in enumerate(legend):
    yy = 16.5 - i * 0.22
    ax.plot([0.5, 1.0], [yy, yy], color=c, linewidth=2.2, zorder=2)
    ax.text(1.1, yy, lbl, fontsize=8.5, va='center', zorder=2)

# =================================================================
# 接線對照表（最右側）
# =================================================================
table_x, table_y = 24.5, 13.8
ax.add_patch(Rectangle((table_x - 0.2, 0.4), 3.5, 13.6,
                       linewidth=1.5, edgecolor=BOX_EDGE, facecolor='#FAFAFA', zorder=1))
ax.text(table_x + 1.5, 13.6, '接線對照表',
        ha='center', fontsize=11, fontweight='bold', zorder=2)
ax.plot([table_x - 0.2, table_x + 3.3], [13.3, 13.3], color=BOX_EDGE, linewidth=1, zorder=2)

connections = [
    ('— I²C 匯流排 —', 'header'),
    ('MPU6050 VCC', '→ 3V3'),
    ('MPU6050 GND', '→ GND'),
    ('MPU6050 SDA', '→ GPIO21'),
    ('MPU6050 SCL', '→ GPIO22'),
    ('BMP280 VCC', '→ 3V3'),
    ('BMP280 SDA', '→ GPIO21'),
    ('BMP280 SCL', '→ GPIO22'),
    ('TCA9548A VCC', '→ 3V3'),
    ('TCA9548A SDA', '→ GPIO21'),
    ('TCA9548A SCL', '→ GPIO22'),
    ('VL53L1X #1', '→ TCA CH0'),
    ('VL53L1X #2', '→ TCA CH1'),
    ('— SPI (NRF24) —', 'header'),
    ('NRF24 VCC', '→ 3V3 +100µF'),
    ('NRF24 GND', '→ GND'),
    ('NRF24 CE', '→ GPIO4'),
    ('NRF24 CSN', '→ GPIO5'),
    ('NRF24 SCK', '→ GPIO18'),
    ('NRF24 MOSI', '→ GPIO23'),
    ('NRF24 MISO', '→ GPIO19'),
    ('— UART (GPS) —', 'header'),
    ('GPS VCC', '→ 5V (BEC)'),
    ('GPS TX', '→ GPIO16 (RX2)'),
    ('GPS RX', '→ GPIO17 (TX2)'),
    ('— PWM (馬達) —', 'header'),
    ('ESC1 SIG', '→ GPIO25'),
    ('ESC2 SIG', '→ GPIO26'),
    ('ESC3 SIG', '→ GPIO27'),
    ('ESC4 SIG', '→ GPIO14'),
    ('ESC + (×4)', '→ +11.1V'),
    ('— 其他 —', 'header'),
    ('電池分壓中點', '→ GPIO34'),
    ('ESP32 VIN', '→ +5V BEC'),
    ('ESP32-CAM 5V', '→ +5V BEC'),
]
y_cursor = 13.0
for label, target in connections:
    if target == 'header':
        ax.text(table_x + 1.5, y_cursor, label, ha='center',
                fontsize=8.5, fontweight='bold', color='#C0392B', zorder=2)
    else:
        ax.text(table_x + 0.0, y_cursor, label, ha='left',
                fontsize=8, color=TEXT, zorder=2)
        ax.text(table_x + 1.6, y_cursor, target, ha='left',
                fontsize=8, color='#2980B9', zorder=2, fontweight='bold')
    y_cursor -= 0.32

# =================================================================
# 標題欄（左下）
# =================================================================
tbox_x, tbox_y = 6, 0.2
ax.add_patch(Rectangle((tbox_x, tbox_y), 6.5, 1.0,
                       linewidth=1.5, edgecolor=BOX_EDGE, facecolor='white', zorder=3))
ax.plot([tbox_x + 2.5, tbox_x + 2.5], [tbox_y, tbox_y + 1.0],
        color=BOX_EDGE, linewidth=1)
ax.text(tbox_x + 1.25, tbox_y + 0.5, '飛控完整電路圖\n（含 GPS、避障、影像）',
        ha='center', va='center', fontsize=9, fontweight='bold')
ax.text(tbox_x + 4.5, tbox_y + 0.6, '版本：v1.0', fontsize=9)
ax.text(tbox_x + 4.5, tbox_y + 0.3, '日期：2026-05', fontsize=9)

plt.savefig('d:/無人機/flightflight/drone_schematic_full.png',
            dpi=130, bbox_inches='tight', facecolor='white')
print("Saved: drone_schematic_full.png")
