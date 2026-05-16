import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.patches import FancyBboxPatch, Rectangle, FancyArrowPatch
import matplotlib

matplotlib.rcParams['font.family'] = ['Microsoft JhengHei', 'sans-serif']

fig, ax = plt.subplots(figsize=(16, 11), dpi=130)
ax.set_xlim(0, 16)
ax.set_ylim(0, 11)
ax.set_aspect('equal')
ax.axis('off')
fig.patch.set_facecolor('white')

# Color palette
ESP32 = '#2C3E50'
SENSOR = '#3498DB'
COMM = '#E67E22'
POWER = '#C0392B'
MOTOR = '#16A085'
TEXT = '#222222'

def block(x, y, w, h, title, lines, fill='#ECF0F1', edge=ESP32, title_color=ESP32):
    box = FancyBboxPatch((x, y), w, h,
                         boxstyle="round,pad=0.02,rounding_size=0.15",
                         linewidth=2, edgecolor=edge, facecolor=fill, zorder=2)
    ax.add_patch(box)
    ax.text(x + w/2, y + h - 0.25, title, ha='center', va='top',
            fontsize=11, fontweight='bold', color=title_color)
    for i, line in enumerate(lines):
        ax.text(x + 0.15, y + h - 0.7 - i*0.32, line,
                ha='left', va='top', fontsize=9, color=TEXT)

def wire(x1, y1, x2, y2, label='', color='#555', lw=1.5, label_offset=(0,0.15)):
    ax.plot([x1, x2], [y1, y2], color=color, linewidth=lw, zorder=1)
    if label:
        mx = (x1 + x2) / 2 + label_offset[0]
        my = (y1 + y2) / 2 + label_offset[1]
        ax.text(mx, my, label, ha='center', va='center',
                fontsize=8, color=color,
                bbox=dict(boxstyle='round,pad=0.15', facecolor='white', edgecolor='none', alpha=0.85))

# Title
ax.text(8, 10.5, '無人機端接線圖 ─ ESP32 飛控',
        ha='center', va='center', fontsize=20, fontweight='bold', color=ESP32)
ax.text(8, 10.05, 'ESP32 DevKit V1 + MPU6050 + BMP280 + GY-NEO6MV2 + VL53L1X×4 + NRF24L01+PA/LNA',
        ha='center', va='center', fontsize=10, color='#666')

# ===== ESP32 in center =====
block(6.5, 4.5, 3.5, 4.5, 'ESP32 DevKit V1', [
    '3V3      GPIO 21 (SDA)',
    'GND      GPIO 22 (SCL)',
    'VIN(5V)  GPIO 16 (RX2)',
    '         GPIO 17 (TX2)',
    '         GPIO 18 (SCK)',
    '         GPIO 19 (MISO)',
    '         GPIO 23 (MOSI)',
    '         GPIO 5  (CSN)',
    '         GPIO 4  (CE)',
    '         GPIO 25 → M1',
    '         GPIO 26 → M2',
    '         GPIO 27 → M3',
    '         GPIO 14 → M4',
    '         GPIO 34 → BatADC',
], fill='#D5DBDB', edge=ESP32, title_color='white')

# Re-color title bar
title_bar = Rectangle((6.5, 8.55), 3.5, 0.5, facecolor=ESP32, zorder=3)
ax.add_patch(title_bar)
ax.text(8.25, 8.8, 'ESP32 DevKit V1', ha='center', va='center',
        fontsize=12, fontweight='bold', color='white', zorder=4)

# ===== I2C bus modules (left side) =====
block(0.3, 7.5, 2.8, 1.6, 'MPU6050 (I²C 0x68)', [
    'VCC → ESP32 3V3',
    'GND → ESP32 GND',
    'SDA → GPIO 21',
    'SCL → GPIO 22',
], fill='#D6EAF8', edge=SENSOR, title_color=SENSOR)

block(0.3, 5.5, 2.8, 1.6, 'BMP280 (I²C 0x76)', [
    'VCC → ESP32 3V3',
    'GND → ESP32 GND',
    'SDA → GPIO 21',
    'SCL → GPIO 22',
], fill='#D6EAF8', edge=SENSOR, title_color=SENSOR)

block(0.3, 3.5, 2.8, 1.6, 'TCA9548A (0x70)', [
    'VCC → ESP32 3V3',
    'GND → ESP32 GND',
    'SDA → GPIO 21',
    'SCL → GPIO 22',
], fill='#D6EAF8', edge=SENSOR, title_color=SENSOR)

# VL53L1X x4
block(0.3, 0.4, 2.8, 2.7, 'VL53L1X × 4 (避障)', [
    '#1 前 → TCA9548A SC0/SD0',
    '#2 右 → TCA9548A SC1/SD1',
    '#3 後 → TCA9548A SC2/SD2',
    '#4 左 → TCA9548A SC3/SD3',
    '',
    'VCC/GND 從 TCA9548A 取',
], fill='#D6EAF8', edge=SENSOR, title_color=SENSOR)

# ===== Right side: SPI / UART =====
block(10.7, 7.5, 4.8, 1.6, 'NRF24L01+PA/LNA (SPI)', [
    'VCC → 3V3（旁邊加 100μF 電容！）',
    'GND → GND      SCK  → GPIO 18',
    'CE  → GPIO 4   MOSI → GPIO 23',
    'CSN → GPIO 5   MISO → GPIO 19',
], fill='#FAE5D3', edge=COMM, title_color=COMM)

block(10.7, 5.5, 4.8, 1.6, 'GY-NEO6MV2 GPS (UART2)', [
    'VCC → 5V (PDB BEC)',
    'GND → GND',
    'TX  → ESP32 GPIO 16 (RX2)',
    'RX  → ESP32 GPIO 17 (TX2)',
], fill='#FAE5D3', edge=COMM, title_color=COMM)

# ESC outputs
block(10.7, 3.4, 4.8, 1.7, 'ESC × 4 → 無刷馬達', [
    'ESC1 訊號 → GPIO 25  (M1 前左 CW)',
    'ESC2 訊號 → GPIO 26  (M2 前右 CCW)',
    'ESC3 訊號 → GPIO 27  (M3 後左 CCW)',
    'ESC4 訊號 → GPIO 14  (M4 後右 CW)',
    'ESC 紅線(5V)不接 ESP32，只接訊號+GND',
], fill='#D1F2EB', edge=MOTOR, title_color=MOTOR)

# Battery ADC
block(10.7, 1.7, 4.8, 1.5, '電池電壓監測', [
    '電池+ ─30kΩ─┬─10kΩ─GND',
    '            └ GPIO 34',
], fill='#FFF5D7', edge='#B7950B', title_color='#B7950B')

# ===== Power section (bottom) =====
block(3.5, 0.4, 6.7, 2.6, '電源架構', [
    '3S LiPo 11.1V (XT60) → PDB',
    '  PDB → 11.1V × 4 → ESC × 4 (馬達動力)',
    '  PDB → 5V BEC ─┬─ ESP32 VIN',
    '              ├─ ESP32-CAM 5V (獨立模組)',
    '              └─ GPS VCC',
    'ESP32 內部 3V3 → MPU6050、BMP280、',
    '                 TCA9548A、VL53L1X、NRF24',
    '⚠️ 所有 GND 必須共地',
], fill='#FADBD8', edge=POWER, title_color=POWER)

# ===== Connection lines =====
# Left modules → ESP32 (I²C bus)
for y_top in [8.3, 6.3, 4.3]:
    wire(3.1, y_top, 6.5, y_top - 0.5, color=SENSOR, lw=1.3)

# TCA9548A → VL53L1X group (indicates same bus)
wire(1.7, 3.5, 1.7, 3.1, color=SENSOR, lw=1.3)

# Right modules → ESP32
wire(10.7, 8.3, 10, 7.5, color=COMM, lw=1.3)         # NRF24
wire(10.7, 6.3, 10, 6.5, color=COMM, lw=1.3)         # GPS
wire(10.7, 4.3, 10, 5.0, color=MOTOR, lw=1.3)        # ESC

# ADC
wire(13, 3.2, 9.5, 4.5, color='#B7950B', lw=1.1)

# Power → ESP32
wire(6.85, 3.0, 6.85, 4.5, color=POWER, lw=2.2, label='5V → VIN')

# Legend
legend_x, legend_y = 0.3, 9.5
ax.text(legend_x, legend_y + 0.3, '線色說明：', fontsize=9, fontweight='bold', color=TEXT)
for i, (color, label) in enumerate([
    (SENSOR, 'I²C 感測器'),
    (COMM, '無線/序列'),
    (MOTOR, '馬達 PWM'),
    (POWER, '電源 5V/3V3'),
    ('#B7950B', '其他 GPIO'),
]):
    ax.plot([legend_x, legend_x + 0.4], [legend_y - i*0.25, legend_y - i*0.25],
            color=color, linewidth=2.5)
    ax.text(legend_x + 0.5, legend_y - i*0.25, label,
            fontsize=8, va='center', color=TEXT)

plt.tight_layout()
out = r"d:\無人機\flightflight\drone_wiring.png"
plt.savefig(out, dpi=140, bbox_inches='tight', facecolor='white')
print(f"Saved: {out}")
