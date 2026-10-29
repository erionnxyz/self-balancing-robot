"""Fig. 1 block diagram for the capstone report (IEEE single column, black and white).

Writes block_diagram.svg and renders block_diagram.png at 600 dpi via headless Chromium.
"""
import os
from playwright.sync_api import sync_playwright
HERE = os.path.dirname(os.path.abspath(__file__))

W, H = 680, 590
FONT = "'Times New Roman', 'Liberation Serif', Times, serif"
T1, T2, TL = 20, 18, 18          # title, sub-line, line-label sizes (SVG units)
THICK, THIN = 3.2, 1.5            # power / signal stroke widths
DASH = '7 4'

el = []


def box(x0, y0, x1, y1, title, sub=None, planned=False):
    fill = '#ffffff' if planned else '#e6e6e6'
    dash = f' stroke-dasharray="{DASH}"' if planned else ''
    el.append(f'<rect x="{x0}" y="{y0}" width="{x1-x0}" height="{y1-y0}" fill="{fill}" '
              f'stroke="#000" stroke-width="1.6"{dash}/>')
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    if sub:
        el.append(text(cx, cy - 4, title, T1, anchor='middle'))
        el.append(text(cx, cy + 19, sub, T2, anchor='middle'))
    else:
        el.append(text(cx, cy + 7, title, T1, anchor='middle'))


def text(x, y, s, size, anchor='start', italic=False, rotate=None):
    st = ' font-style="italic"' if italic else ''
    tr = f' transform="rotate({rotate} {x} {y})"' if rotate else ''
    s = s.replace('&', '&amp;').replace('<', '&lt;')
    return (f'<text x="{x}" y="{y}" font-family="{FONT}" font-size="{size}" '
            f'text-anchor="{anchor}"{st}{tr}>{s}</text>')


def line(pts, width, planned=False, start=False, end=False):
    d = 'M ' + ' L '.join(f'{x} {y}' for x, y in pts)
    dash = f' stroke-dasharray="{DASH}"' if planned else ''
    ms = ' marker-start="url(#a0)"' if start else ''
    me = ' marker-end="url(#a1)"' if end else ''
    el.append(f'<path d="{d}" fill="none" stroke="#000" stroke-width="{width}" '
              f'stroke-linejoin="miter"{dash}{ms}{me}/>')


def dot(x, y):
    el.append(f'<circle cx="{x}" cy="{y}" r="4.2" fill="#000"/>')


# ---- Blocks -----------------------------------------------------------------
box(10, 15, 330, 85, 'Host PC (USB)', 'Arduino IDE, Serial Plotter')
box(530, 15, 670, 85, 'Battery pack', '(planned)', planned=True)

# ESP32 development board (container) with regulator and SoC
el.append('<rect x="10" y="130" width="490" height="145" fill="none" stroke="#000" stroke-width="1.6"/>')
box(20, 150, 190, 225, '3.3 V regulator', '(on board)')
box(245, 150, 485, 225, 'ESP32 SoC', '2 × 240 MHz, FPU')
el.append(text(235, 262, 'ESP32 development board', TL, anchor='middle', italic=True))

# Sensors / display on the shared bus
box(50, 360, 190, 430, 'ADXL345', 'accel., 0x53')
box(205, 360, 355, 430, 'SSD1315 OLED', '128 × 64, 0x3C')
box(370, 360, 500, 430, 'Gyroscope', '(planned)', planned=True)

# Drive train
box(530, 345, 670, 415, 'Motor driver', '(planned)', planned=True)
box(530, 455, 670, 525, '2 × geared DC', 'motors, wheels', planned=True)

# ---- Connections ------------------------------------------------------------
# USB 5 V power into the regulator; USB serial both ways to the SoC
line([(105, 85), (105, 150)], THICK, end=True)
el.append(text(97, 122, 'USB 5 V', TL, anchor='end'))
line([(300, 85), (300, 150)], THIN, start=True, end=True)
el.append(text(308, 122, 'USB serial, 115200 baud', TL))

# Regulator -> SoC (3.3 V)
line([(190, 187), (245, 187)], THICK, end=True)
el.append(text(217, 179, '3.3 V', TL, anchor='middle'))

# 3.3 V rail: from regulator down the left side, under the sensors
line([(35, 225), (35, 475), (280, 475)], THICK)
line([(280, 475), (435, 475)], THICK, planned=True)
line([(120, 475), (120, 430)], THICK, end=True)
line([(280, 475), (280, 430)], THICK, end=True)
line([(435, 475), (435, 430)], THICK, planned=True, end=True)
dot(120, 475)
dot(280, 475)
el.append(text(200, 500, '3.3 V rail (common ground)', TL, anchor='middle'))

# I2C bus: from the SoC down, across the tops of the sensors
line([(435, 225), (435, 320)], THIN, start=True)
line([(435, 320), (120, 320), (120, 360)], THIN, end=True)
line([(280, 320), (280, 360)], THIN, end=True)
line([(435, 320), (435, 360)], THIN, planned=True, end=True)
dot(280, 320)
dot(435, 320)
el.append(text(277, 309, 'I2C: SDA GPIO21, SCL GPIO22, 400 kHz', TL, anchor='middle'))

# PWM to the motor driver
line([(485, 187), (580, 187), (580, 345)], THIN, planned=True, end=True)
el.append(text(533, 179, 'PWM', TL, anchor='middle'))

# Battery -> motor driver (motor supply)
line([(655, 85), (655, 345)], THICK, planned=True, end=True)
el.append(text(648, 245, 'Motor', TL, anchor='end'))
el.append(text(648, 265, 'supply', TL, anchor='end'))

# Motor driver -> motors
line([(600, 415), (600, 455)], THICK, planned=True, end=True)
el.append(text(608, 441, 'L, R', TL))

# ---- Legend -----------------------------------------------------------------
ly = 560
el.append(f'<rect x="20" y="{ly-11}" width="34" height="20" fill="#e6e6e6" stroke="#000" stroke-width="1.4"/>')
el.append(text(62, ly + 6, 'Built (Milestone 1)', TL))
el.append(f'<rect x="232" y="{ly-11}" width="34" height="20" fill="#fff" stroke="#000" stroke-width="1.4" stroke-dasharray="{DASH}"/>')
el.append(text(274, ly + 6, 'Planned', TL))
line([(370, ly), (412, ly)], THICK)
el.append(text(420, ly + 6, 'Power', TL))
line([(490, ly), (532, ly)], THIN)
el.append(text(540, ly + 6, 'Signal / data', TL))

defs = ('<defs>'
        '<marker id="a1" viewBox="0 0 12 10" refX="12" refY="5" markerWidth="12" markerHeight="10" '
        'markerUnits="userSpaceOnUse" orient="auto"><path d="M0 0 L12 5 L0 10 z" fill="#000"/></marker>'
        '<marker id="a0" viewBox="0 0 12 10" refX="12" refY="5" markerWidth="12" markerHeight="10" '
        'markerUnits="userSpaceOnUse" orient="auto-start-reverse"><path d="M0 0 L12 5 L0 10 z" fill="#000"/></marker>'
        '</defs>')

svg = (f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}">'
       f'<rect width="{W}" height="{H}" fill="#fff"/>{defs}{"".join(el)}</svg>')
open(os.path.join(HERE, 'Fig1_Block_Diagram.svg'), 'w').write(svg)

# Render: final print width 3.45 in at 600 dpi -> 2070 px
scale = 2070 / W
with sync_playwright() as p:
    b = p.chromium.launch()
    pg = b.new_page(viewport={'width': W, 'height': H}, device_scale_factor=scale)
    pg.set_content(f'<html><body style="margin:0">{svg}</body></html>')
    pg.wait_for_timeout(300)
    pg.screenshot(path=os.path.join(HERE, 'Fig1_Block_Diagram.png'), clip={'x': 0, 'y': 0, 'width': W, 'height': H})
    b.close()
print('ok', W, H, 'px', round(W * scale), round(H * scale))
