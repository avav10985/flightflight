"""
地面站接線圖 — 38-pin ESP32 + 雙搖桿 + NRF24 + LCD 2004A
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
WIRE_ADC = '#27AE60'
WIRE_SPI = '#8E44AD'
WIRE_I2C = '#2980B9'
WIRE_PWR = '#C0392B'
WIRE_GND = '#000'
WIRE_DIG = '#E67E22'
TEXT = '#111'

ax.text(12, 15.4, '地面站接線圖 ─ 38-pin ESP32', ha='center',
        fontsize=22, fontweight='bold', color=ESP)
ax.text(12, 14.9, 'ESP32 + 雙搖桿(Mode 2) + NRF24L01+PA/LNA + LCD 2004A',
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


# ===== ESP32 中央 =====
box(9, 4.5, 5.5, 8.5, 'ESP32\n(38-pin)', [
    'GPIO34 ← 油門(左上下)',
    'GPIO35 ← yaw (左左右)',
    'GPIO32 ← pitch(右上下)',
    'GPIO33 ← roll (右左右)',
    'GPIO25 ← 武裝開關',
    'GPIO26 ← 選單開關',
    'GPIO21 → LCD SDA',
    'GPIO22 → LCD SCL',
    'GPIO18 → NRF SCK',
    'GPIO19 → NRF MISO',
    'GPIO23 → NRF MOSI',
    'GPIO5  → NRF CSN',
    'GPIO4  → NRF CE',
    '3V3 / 5V / GND',
], fill='#D5DBDB', tc='white')
ax.add_patch(Rectangle((9, 12.5), 5.5, 0.5, facecolor=ESP, zorder=3))
ax.text(11.75, 12.75, 'ESP32 (38-pin)', ha='center', va='center',
        fontsize=12, fontweight='bold', color='white', zorder=4)

# ===== 左搖桿 =====
box(0.5, 11, 4, 3, '左搖桿', [
    'VRy(上下) → GPIO34  油門',
    'VRx(左右) → GPIO35  yaw',
    'VCC → 3V3',
    'GND → GND',
], fill='#D6EAF8', edge=WIRE_ADC, tc=WIRE_ADC)

# ===== 右搖桿 =====
box(0.5, 7.5, 4, 3, '右搖桿', [
    'VRy(上下) → GPIO32  pitch',
    'VRx(左右) → GPIO33  roll',
    'VCC → 3V3',
    'GND → GND',
], fill='#D6EAF8', edge=WIRE_ADC, tc=WIRE_ADC)

# ===== 開關 =====
box(0.5, 4.7, 4, 2.3, '開關 × 2', [
    'SW1 武裝 → GPIO25',
    'SW2 選單 → GPIO26',
    '另一腳 → GND',
    '(內部 PULLUP)',
], fill='#FDEBD0', edge=WIRE_DIG, tc=WIRE_DIG)

# ===== NRF24 =====
box(17, 9.5, 6, 4, 'NRF24L01+PA/LNA', [
    'VCC → 3V3（+100µF 電容！）',
    'GND → GND      CE  → GPIO4',
    'CSN → GPIO5    SCK → GPIO18',
    'MOSI→ GPIO23   MISO→ GPIO19',
    'IRQ → 不接',
], fill='#EBDEF0', edge=WIRE_SPI, tc=WIRE_SPI)

# ===== LCD =====
box(17, 5, 6, 3.5, 'LCD 2004A (I²C)', [
    'VCC → 5V',
    'GND → GND',
    'SDA → GPIO21',
    'SCL → GPIO22',
    '(PCF8574 背板 0x27)',
], fill='#D5F5E3', edge=WIRE_I2C, tc=WIRE_I2C)

# ===== 連線 =====
# 搖桿 → ESP32 (ADC)
wire((4.5, 12.8), (9, 12.3), WIRE_ADC, 'GPIO34')
wire((4.5, 12.2), (9, 11.9), WIRE_ADC, 'GPIO35')
wire((4.5, 9.3),  (9, 11.5), WIRE_ADC, 'GPIO32')
wire((4.5, 8.7),  (9, 11.1), WIRE_ADC, 'GPIO33')
# 開關 → ESP32
wire((4.5, 6.3), (9, 10.3), WIRE_DIG, 'GPIO25')
wire((4.5, 5.8), (9, 9.9), WIRE_DIG, 'GPIO26')
# ESP32 → LCD (I2C)
wire((14.5, 9.5), (17, 7.0), WIRE_I2C, 'SDA/SCL')
# ESP32 → NRF24 (SPI)
wire((14.5, 11.5), (17, 11.5), WIRE_SPI, 'SPI×5')

# ===== 電源說明 =====
box(9, 0.5, 9, 3.3, '電源', [
    '行動電源 / 充電器 → ESP32 USB 孔（5V）',
    'ESP32 板上 3.3V → 搖桿 VCC、NRF24 VCC',
    'ESP32 5V (VIN) → LCD VCC',
    '所有 GND 共地',
    '⚠️ NRF24 必須 3.3V（5V 會燒）',
], fill='#FADBD8', edge=WIRE_PWR, tc=WIRE_PWR)

# ===== 圖例 =====
lx, ly = 19, 3.8
ax.add_patch(Rectangle((lx - 0.2, ly - 1.7), 4.5, 2.0, linewidth=1,
             edgecolor='gray', facecolor='#F8F9FA', alpha=0.9, zorder=1))
ax.text(lx, ly, '線色：', fontsize=9, fontweight='bold', zorder=2)
for i, (c, l) in enumerate([(WIRE_ADC, '搖桿 ADC'), (WIRE_DIG, '開關'),
                            (WIRE_SPI, 'NRF24 SPI'), (WIRE_I2C, 'LCD I²C'),
                            (WIRE_PWR, '電源')]):
    yy = ly - 0.35 - i*0.28
    ax.plot([lx, lx + 0.5], [yy, yy], color=c, linewidth=2.5, zorder=2)
    ax.text(lx + 0.65, yy, l, fontsize=8.5, va='center', zorder=2)

plt.savefig('d:/無人機/flightflight/ground_wiring.png',
            dpi=130, bbox_inches='tight', facecolor='white')
print("Saved: ground_wiring.png")
