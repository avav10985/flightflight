"""
飛機端接線圖 — LOLIN D32 + MPU6050 + NRF24 + ESC×4 + 電池監測
"""
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle, FancyBboxPatch
import matplotlib

matplotlib.rcParams['font.family'] = ['Microsoft JhengHei', 'sans-serif']
matplotlib.rcParams['axes.unicode_minus'] = False

fig, ax = plt.subplots(figsize=(16, 11), dpi=130)
ax.set_xlim(0, 24)
ax.set_ylim(0, 16)
ax.set_aspect('equal')
ax.axis('off')
fig.patch.set_facecolor('white')

ESP = '#2C3E50'
WIRE_I2C = '#2980B9'
WIRE_SPI = '#8E44AD'
WIRE_PWM = '#16A085'
WIRE_PWR = '#C0392B'
WIRE_GND = '#000'
WIRE_ADC = '#27AE60'
TEXT = '#111'

ax.text(12, 15.4, '飛機端接線圖 ─ LOLIN D32', ha='center',
        fontsize=22, fontweight='bold', color=ESP)
ax.text(12, 14.9, 'LOLIN D32 + MPU6050 + NRF24L01+PA/LNA + ESC×4 + 電池監測',
        ha='center', fontsize=11, color='gray', style='italic')


def box(x, y, w, h, title, lines, fill='#ECF0F1', edge=ESP, tc=ESP):
    ax.add_patch(FancyBboxPatch((x, y), w, h,
                 boxstyle="round,pad=0.02,rounding_size=0.15",
                 linewidth=2, edgecolor=edge, facecolor=fill, zorder=2))
    ax.text(x + w/2, y + h - 0.3, title, ha='center', va='top',
            fontsize=11, fontweight='bold', color=tc)
    for i, ln in enumerate(lines):
        ax.text(x + 0.2, y + h - 0.85 - i*0.42, ln, ha='left', va='top',
                fontsize=9.5, color=TEXT)


def wire(p1, p2, color, label='', lw=1.6):
    x1, y1 = p1; x2, y2 = p2
    ax.plot([x1, (x1+x2)/2, (x1+x2)/2, x2], [y1, y1, y2, y2],
            color=color, linewidth=lw, zorder=1)
    ax.add_patch(Circle((x1, y1), 0.07, color=color, zorder=3))
    ax.add_patch(Circle((x2, y2), 0.07, color=color, zorder=3))
    if label:
        ax.text((x1+x2)/2, (y1+y2)/2 + 0.18, label, ha='center', fontsize=8,
                color=color, bbox=dict(boxstyle='round,pad=0.12',
                facecolor='white', edgecolor=color, linewidth=0.5), zorder=4)


# ===== LOLIN D32 中央 =====
box(9, 4, 5.5, 9, 'LOLIN D32', [
    'GPIO21 → MPU6050 SDA',
    'GPIO22 → MPU6050 SCL',
    'GPIO18 → NRF SCK',
    'GPIO19 → NRF MISO',
    'GPIO23 → NRF MOSI',
    'GPIO5  → NRF CSN',
    'GPIO4  → NRF CE',
    'GPIO25 → ESC1 (前左)',
    'GPIO26 → ESC2 (前右)',
    'GPIO27 → ESC3 (後左)',
    'GPIO14 → ESC4 (後右)',
    'GPIO34 ← 電池分壓',
    'USB(5V) ← ESC BEC',
    '3V3 → 模組電源',
], fill='#D5DBDB', tc='white')
ax.add_patch(Rectangle((9, 12.5), 5.5, 0.5, facecolor=ESP, zorder=3))
ax.text(11.75, 12.75, 'LOLIN D32', ha='center', va='center',
        fontsize=12, fontweight='bold', color='white', zorder=4)

# ===== MPU6050 =====
box(0.5, 11, 4, 3, 'MPU6050 (I²C)', [
    'VCC → 3V3',
    'GND → GND',
    'SDA → GPIO21',
    'SCL → GPIO22',
], fill='#D6EAF8', edge=WIRE_I2C, tc=WIRE_I2C)

# ===== NRF24 =====
box(0.5, 6.8, 4, 3.8, 'NRF24+PA/LNA', [
    'VCC→3V3 +100µF',
    'GND → GND',
    'CE → GPIO4',
    'CSN → GPIO5',
    'SCK → GPIO18',
    'MOSI→ GPIO23',
    'MISO→ GPIO19',
], fill='#EBDEF0', edge=WIRE_SPI, tc=WIRE_SPI)

# ===== 電池監測 =====
box(0.5, 4, 4, 2.5, '電池監測', [
    '電池+ → 30kΩ → A',
    'A → GPIO34',
    'A → 10kΩ → GND',
    '(分壓 1/4)',
], fill='#FCF3CF', edge=WIRE_ADC, tc='#9A7D0A')

# ===== ESC × 4 =====
box(17, 8.5, 6.5, 5, 'ESC × 4 → 無刷馬達', [
    'ESC1 訊號 → GPIO25  (M1 前左 CW)',
    'ESC2 訊號 → GPIO26  (M2 前右 CCW)',
    'ESC3 訊號 → GPIO27  (M3 後左 CCW)',
    'ESC4 訊號 → GPIO14  (M4 後右 CW)',
    '',
    'ESC1 紅(5V BEC) → USB 腳供電',
    'ESC2~4 紅線 → 包熱縮不接',
    '4 顆黑線(GND) → 共地',
], fill='#D1F2EB', edge=WIRE_PWM, tc=WIRE_PWM)

# ===== 連線 =====
wire((4.5, 12.8), (9, 12.3), WIRE_I2C, 'SDA/SCL')
wire((4.5, 8.7),  (9, 11.0), WIRE_SPI, 'SPI×5')
wire((4.5, 5.2),  (9, 9.0),  WIRE_ADC, 'GPIO34')
wire((14.5, 11.0), (17, 11.0), WIRE_PWM, 'PWM×4')

# ===== 電源 =====
box(9, 0.5, 9, 3, '電源', [
    '3S 電池 → PDB → 11.1V × 4 → ESC 動力',
    'ESC1 BEC 5V → LOLIN USB 腳',
    'LOLIN 3V3 → MPU6050 / NRF24',
    '電池 11.1V → 30k/10k 分壓 → GPIO34',
    '所有 GND 共地',
], fill='#FADBD8', edge=WIRE_PWR, tc=WIRE_PWR)

# ===== 圖例 =====
lx, ly = 19, 4.0
ax.add_patch(Rectangle((lx - 0.2, ly - 1.9), 4.5, 2.2, linewidth=1,
             edgecolor='gray', facecolor='#F8F9FA', alpha=0.9, zorder=1))
ax.text(lx, ly, '線色：', fontsize=9, fontweight='bold', zorder=2)
for i, (c, l) in enumerate([(WIRE_I2C, 'I²C (MPU6050)'), (WIRE_SPI, 'SPI (NRF24)'),
                            (WIRE_PWM, 'PWM (ESC)'), (WIRE_ADC, 'ADC (電池)'),
                            (WIRE_PWR, '電源'), (WIRE_GND, 'GND')]):
    yy = ly - 0.35 - i*0.28
    ax.plot([lx, lx + 0.5], [yy, yy], color=c, linewidth=2.5, zorder=2)
    ax.text(lx + 0.65, yy, l, fontsize=8.5, va='center', zorder=2)

plt.savefig('d:/無人機/flightflight/drone_wiring2.png',
            dpi=130, bbox_inches='tight', facecolor='white')
print("Saved: drone_wiring2.png")
