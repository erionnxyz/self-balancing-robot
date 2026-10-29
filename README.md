# Self-Balancing Robot (EE 449/449L Capstone, Fall 2026)

Capstone project for EE 449/449L at the University of Scranton, Department of Physics and Engineering.
Team 8: Erion Kraja and Yoshihiko Matsui.

The project began as a two-wheeled self-balancing robot. It now centers on a **reaction-wheel balancer**:
a single heavy reaction wheel balances a triangular frame on one axis. The first goal is vertical
balance (the triangle standing on one point). A later-stage idea is to change the balance point on
demand, so the frame holds itself at an angle.

## Status

| Milestone | Goal | Status |
|---|---|---|
| 1 | ESP32 reads the ADXL345 accelerometer over I2C and shows tilt (pitch/roll) on an SSD1315 OLED | Firmware written; see `firmware/milestone1_tilt_display` |
| 2 | NEMA 17 stepper (DRV8825 driver) turns opposite to the roll angle, smoothly and without jitter | Working; see `firmware/milestone2_tilt_motor` |
| 3 | Move the circuit (without the OLED) into KiCad and design a custom PCB | In progress; see `hardware/` |
| Next | Reaction-wheel balance of the triangular frame (sensor fusion + closed-loop control) | Planned |

## Repository layout

```
firmware/
  milestone1_tilt_display/   ESP32 + ADXL345 + OLED tilt display (Arduino sketch)
  milestone2_tilt_motor/     Milestone 1 + DRV8825/NEMA 17 stepper that mirrors roll
hardware/                    KiCad project and PCB files (to be added)
docs/
  report/                    Weekly IEEE-format capstone report (Submission 1, Week 1)
  figures/                   Block diagram (PNG + SVG) and the script that draws it
```

Each Arduino sketch sits in a folder with the same name, as the Arduino IDE requires.

## Hardware

| Part | Role |
|---|---|
| ESP32 development board ("ESP32 Dev Module") | Main controller |
| ADXL345 3-axis accelerometer (I2C, address 0x53) | Tilt sensing |
| SSD1315 128 x 64 OLED (I2C, address 0x3C) | Local display (optional from Milestone 2 on) |
| TI DRV8825 stepper driver | Motor driver (1/32 microstepping) |
| NEMA 17 stepper motor (1.8 degree) | Actuator |
| 12 V supply | Motor power (VMOT) |

### Wiring

I2C bus (Milestones 1 and 2):

| ESP32 | ADXL345 | OLED | Function |
|---|---|---|---|
| 3V3 | VCC | VCC | 3.3 V supply |
| GND | GND | GND | Common ground |
| GPIO21 | SDA | SDA | I2C data (400 kHz) |
| GPIO22 | SCL | SCL | I2C clock |
| 3V3 | CS | - | Selects I2C mode |
| GND | SDO | - | Sets address 0x53 |

Stepper (Milestone 2):

| ESP32 / supply | DRV8825 | Notes |
|---|---|---|
| GPIO25 | STEP | Hardware-generated step pulses |
| GPIO26 | DIR | |
| GPIO27 | EN | 10 k pull-up to 3V3 keeps the driver off during boot |
| 3V3 | RST, SLP, M0, M1, M2 | 1/32 microstepping |
| 12 V | VMOT | 100 uF capacitor across VMOT/GND |
| GND | GND | Common ground |

NEMA 17 coils: black -> A1, green -> A2, red -> B1, blue -> B2.

## Building the firmware

1. Install the Arduino IDE and the **esp32 by Espressif Systems** board package. Select **ESP32 Dev Module**.
2. Install the libraries:
   - **Adafruit SSD1306** (also installs Adafruit GFX and Adafruit BusIO)
   - **FastAccelStepper** by Jochen Kiemes (Milestone 2 only)
3. Open the sketch folder, upload, and open the Serial Monitor at **115200 baud**.

Both sketches print staged start-up diagnostics (I2C line check, bus scan, OLED and ADXL345
detection, configuration), then stream live data in `name:value` format for the Arduino Serial
Plotter. In Milestone 2, type `z` and press Enter to set the current angle as the motor's zero.

Milestone 2 tuning constants (gain, deadband, speed and acceleration limits, shake rejection, idle
coil shut-off) are grouped near the top of `milestone2_tilt_motor.ino`.

## Documentation

- `docs/report/Capstone_EE449L_Report_Week1.docx`: the IEEE-format progress report as of Submission 1 (Week 1).
  Sections still marked `[FILL IN]` are completed in later weekly submissions.
- `docs/figures/Fig1_Block_Diagram.png` / `.svg`: the Milestone 1 block diagram (Fig. 1 of the report).
  `make_block_diagram.py` regenerates it (requires Python and Playwright).

The Week 1 report and block diagram describe the original two-wheel design. Later reports cover the
change to the stepper and reaction-wheel design.

## Acknowledgment of AI assistance

This project used a large language model (LLM), **Anthropic's Claude**, as a development aid.
Claude assisted with:

- drafting the Milestone 1 and Milestone 2 Arduino firmware in `firmware/`
- drafting text, tables, and datasheet summaries in the Week 1 report in `docs/report/`
- drawing the block diagram in `docs/figures/` and writing the script that generates it
- writing this README

The team reviewed, wired, tested, and debugged the hardware and code, and is responsible for all
design decisions and for the accuracy of what is submitted. Code and files that the team wrote
without AI assistance are added to this repository separately by the team. AI assistance is also
cited in the report's references, as course policy requires.

## References

- Analog Devices, *ADXL345 Data Sheet*, Rev. G.
- Espressif Systems, *ESP32 Series Datasheet*.
- Solomon Systech, *SSD1315 Data Sheet*.
- Texas Instruments, *DRV8825 Stepper Motor Controller IC Data Sheet*.
- NXP, *UM10204 I2C-bus Specification and User Manual*.
- [Arduino core for ESP32](https://github.com/espressif/arduino-esp32)
- [Adafruit_SSD1306](https://github.com/adafruit/Adafruit_SSD1306) and [Adafruit GFX](https://github.com/adafruit/Adafruit-GFX-Library)
- [FastAccelStepper](https://github.com/gin66/FastAccelStepper)
