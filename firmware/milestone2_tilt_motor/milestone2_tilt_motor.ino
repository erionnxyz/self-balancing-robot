/*
  Milestone 2: ESP32 + ADXL345 + SSD1315 OLED + DRV8825 / NEMA 17 stepper
  ------------------------------------------------------------------------
  The OLED is optional: if none answers at startup, the sketch runs without
  it and everything still prints to the Serial Monitor.

  Everything from milestone 1 is unchanged: the ADXL345 is read at 100 Hz,
  tilt (pitch P and roll R) is worked out from gravity, and the OLED shows
  the same numbers and bubble level.

  New: the stepper mirrors R, amplified. When R increases, the shaft turns
  the other way by MOTOR_GAIN times as many degrees (3x by default), so small
  tilts give bigger turns. The angle R had at power-up is the motor's zero,
  so nothing moves until you tilt the board.

  How the motion is kept smooth and free of jitter:
    1. A motor-only filter smooths R further (the display is not affected).
    2. A deadband ignores wobble smaller than DEADBAND_DEG, so a still board
       gives a still motor.
    3. Speed and acceleration limits turn every change into a smooth ramp.
    4. Step pulses come from ESP32 hardware (FastAccelStepper library), so
       screen updates and I2C reads never make the motor stutter.
    5. While the board is being shaken or bumped (|a| far from 1 g), or held
       where R is meaningless, the motor holds still instead of chasing
       false readings.
    6. After the motor has been still for a moment, the driver switches its
       coils off, which silences the DRV8825's holding hum. It switches back
       on before the next move.

  Wiring (see the milestone 2 schematic):
    I2C, unchanged : GPIO21 = SDA, GPIO22 = SCL, ADXL345 + OLED on 3V3/GND
    DRV8825        : STEP <- GPIO25, DIR <- GPIO26, EN <- GPIO27 (+10k to 3V3)
                     RST, SLP, M0, M1, M2 -> 3V3 (1/32 microstepping)
                     VMOT <- 12 V (100 uF across VMOT/GND), GND -> common ground
    NEMA 17        : black -> A1, green -> A2, red -> B1, blue -> B2

  Arduino IDE setup:
    Board package : "esp32 by Espressif Systems" -> Board: "ESP32 Dev Module"
    Libraries     : "Adafruit SSD1306" (also installs Adafruit GFX + BusIO)
                    "FastAccelStepper" by Jochen Kiemes (gin66)
    Serial Monitor: 115200 baud. Type z and press Enter to make the current
                    angle the new zero (only while the motor is still).

  The SSD1315 uses the same command set as the SSD1306, which is why the
  Adafruit SSD1306 library drives it without changes.
*/

#include <Wire.h>
#include <math.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <FastAccelStepper.h>

// ---------------- Pins and timing ----------------
constexpr int      PIN_SDA      = 21;      // default ESP32 I2C pins, not strapping pins
constexpr int      PIN_SCL      = 22;
constexpr uint32_t I2C_HZ       = 400000;  // I2C fast mode; both chips support 400 kHz

constexpr int      SCREEN_W     = 128;
constexpr int      SCREEN_H     = 64;

constexpr uint32_t SAMPLE_MS    = 10;      // read sensor at 100 Hz (matches ADXL345 data rate)
constexpr uint32_t DISPLAY_MS   = 50;      // redraw screen at 20 Hz
constexpr float    FILTER_ALPHA = 0.2f;    // smoothing 0..1: lower = smoother but slower to react

// ---------------- ADXL345 registers (datasheet Table 19) ----------------
constexpr uint8_t ADXL_REG_DEVID       = 0x00;  // reads 0xE5 on a real ADXL345
constexpr uint8_t ADXL_REG_BW_RATE     = 0x2C;
constexpr uint8_t ADXL_REG_POWER_CTL   = 0x2D;
constexpr uint8_t ADXL_REG_DATA_FORMAT = 0x31;
constexpr uint8_t ADXL_REG_DATAX0      = 0x32;  // X0,X1,Y0,Y1,Z0,Z1 follow in order
constexpr uint8_t ADXL_DEVID_VALUE     = 0xE5;
constexpr float   ADXL_G_PER_LSB       = 0.0039f; // full-resolution mode: 3.9 mg per count

// ---------------- Stepper motor (DRV8825 + NEMA 17) ----------------
constexpr int   PIN_STEP = 25;
constexpr int   PIN_DIR  = 26;
constexpr int   PIN_EN   = 27;           // LOW = driver on. R3 holds it HIGH (off) during boot.

constexpr int   FULL_STEPS_PER_REV = 200;   // 1.8 degree NEMA 17
constexpr int   MICROSTEPS         = 32;    // M0, M1, M2 all tied to 3.3 V
constexpr float STEPS_PER_DEG      = FULL_STEPS_PER_REV * MICROSTEPS / 360.0f;  // 17.78

// Tuning. These are the only numbers you should need to touch.
constexpr int   MOTOR_DIRECTION   = -1;      // -1 mirrors R. If the shaft turns the SAME way as R, change to +1.
constexpr float MOTOR_GAIN        = 3.0f;    // shaft degrees per degree of board tilt (1 = exact mirror)
constexpr float MOTOR_FILTER      = 0.15f;   // extra smoothing for the motor only, 0..1: lower = calmer but more lag
constexpr float DEADBAND_DEG      = 0.5f;    // board wobble smaller than this is ignored (raise if the motor still creeps)
constexpr float MAX_SPEED_DEG_S   = 1080.0f; // top shaft speed in degrees per second (180 rpm)
constexpr float MAX_ACCEL_DEG_S2  = 6000.0f; // ramp steepness: lower = gentler starts and stops, but more lag
constexpr float SHAKE_LIMIT_G     = 0.25f;   // hold the motor while |a| is further than this from 1 g
constexpr float MIN_ROLL_PLANE_G  = 0.35f;   // hold the motor when the board is near vertical and R is undefined
constexpr bool  QUIET_WHEN_IDLE   = true;    // switch the coils off when still (silent, but no holding torque)
constexpr uint16_t IDLE_OFF_MS    = 500;     // how long the motor must be still before the coils switch off

FastAccelStepperEngine engine;
FastAccelStepper *stepper = nullptr;

float   lastRollDeg     = 0;     // last trusted R, used to remove the +/-180 degree jump
float   rollContinuous  = 0;     // R without the jump (can go past 180)
float   rollSmoothed    = 0;     // rollContinuous after the motor-only filter
float   rollHeld        = 0;     // R after the deadband
float   rollZero        = 0;     // R that corresponds to motor position 0
int32_t motorTargetSteps = 0;
bool    motorHolding    = false; // true while the reading can't be trusted

// Third and fourth arguments keep the bus at 400 kHz before and after each screen update.
// (By default the library drops the bus to 100 kHz after drawing.)
Adafruit_SSD1306 display(SCREEN_W, SCREEN_H, &Wire, -1, I2C_HZ, I2C_HZ);

uint8_t adxlAddr = 0;   // filled in by findAdxl()
uint8_t oledAddr = 0;   // stays 0 when no OLED is connected
bool    hasOled  = false;

// Smoothed acceleration (g) and tilt angles (degrees)
float ax = 0, ay = 0, az = 1;
float pitchDeg = 0, rollDeg = 0;
uint32_t readErrors = 0;

// ---------------- Low-level I2C helpers ----------------
bool i2cPresent(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;      // 0 means the device ACKed its address
}

bool writeReg(uint8_t addr, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool readRegs(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission(false);             // repeated start, keep the bus
  if (Wire.requestFrom(addr, len) != len) return false;
  for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

// ---------------- ADXL345 ----------------
uint8_t findAdxl() {
  const uint8_t candidates[] = {0x53, 0x1D}; // SDO low, SDO high
  for (uint8_t a : candidates) {
    uint8_t id = 0;
    if (readRegs(a, ADXL_REG_DEVID, &id, 1) && id == ADXL_DEVID_VALUE) return a;
  }
  return 0;
}

bool adxlInit() {
  // Datasheet recommends configuring in standby, then switching to measurement.
  return writeReg(adxlAddr, ADXL_REG_POWER_CTL,   0x00) &&  // standby
         writeReg(adxlAddr, ADXL_REG_BW_RATE,     0x0A) &&  // 100 Hz output rate, normal power
         writeReg(adxlAddr, ADXL_REG_DATA_FORMAT, 0x09) &&  // FULL_RES=1, range +/-4 g, right-justified
         writeReg(adxlAddr, ADXL_REG_POWER_CTL,   0x08);    // Measure bit on
}

bool adxlReadG(float &x, float &y, float &z) {
  uint8_t b[6];
  // One burst read of all six bytes so X, Y, Z come from the same sample.
  if (!readRegs(adxlAddr, ADXL_REG_DATAX0, b, 6)) return false;
  int16_t rx = (int16_t)((b[1] << 8) | b[0]);  // low byte first, two's complement
  int16_t ry = (int16_t)((b[3] << 8) | b[2]);
  int16_t rz = (int16_t)((b[5] << 8) | b[4]);
  x = rx * ADXL_G_PER_LSB;
  y = ry * ADXL_G_PER_LSB;
  z = rz * ADXL_G_PER_LSB;
  return true;
}

void updateTilt() {
  // Tilt from gravity. Only accurate while the sensor isn't accelerating.
  rollDeg  = atan2f(ay, az) * RAD_TO_DEG;
  pitchDeg = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;
}

// ---------------- Stepper ----------------
// Called after every new sensor sample. Turns R into a motor target.
void updateMotorTarget() {
  // 1. Only trust R when the board is roughly still and R is defined.
  float mag       = sqrtf(ax * ax + ay * ay + az * az);
  float rollPlane = sqrtf(ay * ay + az * az);   // gravity in the plane R is measured in
  motorHolding = fabsf(mag - 1.0f) > SHAKE_LIMIT_G || rollPlane < MIN_ROLL_PLANE_G;
  if (motorHolding) return;                     // keep the last target; resume when trustworthy

  // 2. Remove the jump from +180 to -180 so the motor never spins a full turn the wrong way.
  float delta = rollDeg - lastRollDeg;
  if (delta > 180.0f)  delta -= 360.0f;
  if (delta < -180.0f) delta += 360.0f;
  rollContinuous += delta;
  lastRollDeg = rollDeg;

  // 3. Extra smoothing for the motor only (the display keeps its own, lighter filter).
  rollSmoothed += MOTOR_FILTER * (rollContinuous - rollSmoothed);

  // 4. Deadband: the held angle only moves once R has moved DEADBAND_DEG past it,
  //    then follows R continuously. Tiny noise never reaches the motor.
  if (rollSmoothed > rollHeld + DEADBAND_DEG)      rollHeld = rollSmoothed - DEADBAND_DEG;
  else if (rollSmoothed < rollHeld - DEADBAND_DEG) rollHeld = rollSmoothed + DEADBAND_DEG;

  // 5. Mirror and amplify: 1 degree of R = MOTOR_GAIN degrees of shaft, opposite direction.
  int32_t target = lroundf(MOTOR_DIRECTION * MOTOR_GAIN * (rollHeld - rollZero) * STEPS_PER_DEG);
  if (target != motorTargetSteps) {
    motorTargetSteps = target;
    stepper->moveTo(target);   // the library re-plans a smooth ramp toward the new target
  }
}

float motorAngleDeg() {
  return stepper ? stepper->getCurrentPosition() / STEPS_PER_DEG : 0.0f;
}

// Type z in the Serial Monitor to make the current angle the new zero.
void handleSerialCommands() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c != 'z' && c != 'Z') continue;
    if (stepper->isRunning()) {
      Serial.println("# not re-zeroed: wait until the motor has stopped");
    } else {
      stepper->setCurrentPosition(0);
      motorTargetSteps = 0;
      rollZero = rollHeld;
      Serial.println("# re-zeroed: this angle is now the motor's zero");
    }
  }
}

// ---------------- Display ----------------
void showMessage(const char *line1, const char *line2, const char *line3) {
  if (!hasOled) return;                    // no screen: messages go to the Serial Monitor only
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);   display.println(line1);
  display.setCursor(0, 16);  display.println(line2);
  display.setCursor(0, 28);  display.println(line3);
  display.display();
}

void printDegrees(float value) {
  display.printf("%+6.1f", value);
  display.write(248);                      // degree symbol in the CP437 font
}

void drawScreen() {
  if (!hasOled) return;
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // Left half: numbers
  display.setCursor(0, 0);   display.printf("X:%+5.2f g", ax);
  display.setCursor(0, 10);  display.printf("Y:%+5.2f g", ay);
  display.setCursor(0, 20);  display.printf("Z:%+5.2f g", az);
  display.setCursor(0, 36);  display.print("P:"); printDegrees(pitchDeg);
  display.setCursor(0, 46);  display.print("R:"); printDegrees(rollDeg);

  // Total acceleration: ~1.00 g when still. If it isn't, the tilt reading is unreliable.
  float mag = sqrtf(ax * ax + ay * ay + az * az);
  display.setCursor(0, 56);  display.printf("|a|:%4.2f g", mag);

  // Right half: bubble level. Full deflection = 45 degrees.
  const int cx = 100, cy = 32, r = 27;
  display.drawCircle(cx, cy, r, SSD1306_WHITE);
  display.drawCircle(cx, cy, 9, SSD1306_WHITE);   // inner ring ~ +/-15 degrees
  display.drawFastHLine(cx - r, cy, 2 * r + 1, SSD1306_WHITE);
  display.drawFastVLine(cx, cy - r, 2 * r + 1, SSD1306_WHITE);

  const float maxDeg = 45.0f;
  const int travel = r - 4;
  int dx = (int)(constrain(rollDeg,  -maxDeg, maxDeg) / maxDeg * travel);
  int dy = (int)(constrain(pitchDeg, -maxDeg, maxDeg) / maxDeg * travel);
  display.fillCircle(cx + dx, cy + dy, 4, SSD1306_WHITE);

  display.display();
}

// ---------------- Setup and loop ----------------
// Print every device that answers on the bus. Expect 0x53 (ADXL345), plus 0x3C if an OLED is connected.
void scanBus() {
  int found = 0;
  for (uint8_t a = 1; a < 127; a++) {
    if (i2cPresent(a)) {
      Serial.printf("    device at 0x%02X\n", a);
      found++;
    }
  }
  if (found == 0) Serial.println("    nothing found: check SDA/SCL, ground, power, pull-ups");
}

void setup() {
  Serial.begin(115200);
  delay(2000);  // give the Serial Monitor time to catch up so no early messages are missed
  Serial.println("\n=== ESP32 + ADXL345 + SSD1315 tilt display + stepper ===");

  // Keep the driver switched off until everything is ready (R3 also does this in hardware).
  pinMode(PIN_EN, OUTPUT);
  digitalWrite(PIN_EN, HIGH);

  // [1] Idle level of the I2C lines before I2C starts. Both should be HIGH (pulled up).
  pinMode(PIN_SDA, INPUT);
  pinMode(PIN_SCL, INPUT);
  delay(5);
  Serial.printf("[1] Idle line check: SDA(GPIO%d)=%s  SCL(GPIO%d)=%s\n",
                PIN_SDA, digitalRead(PIN_SDA) ? "HIGH" : "LOW",
                PIN_SCL, digitalRead(PIN_SCL) ? "HIGH" : "LOW");
  Serial.println("    (LOW means no pull-up on that line, or something is holding it low)");

  // [2] Start I2C and scan
  Serial.println("[2] Starting I2C and scanning...");
  Wire.begin(PIN_SDA, PIN_SCL, I2C_HZ);
  scanBus();

  // [3] OLED is optional: use it if one answers, otherwise carry on without it.
  Serial.println("[3] Checking for an OLED at 0x3C / 0x3D (optional)...");
  if (i2cPresent(0x3C))      oledAddr = 0x3C;
  else if (i2cPresent(0x3D)) oledAddr = 0x3D;

  // periphBegin=false: we already started Wire on our pins above.
  if (oledAddr != 0 && display.begin(SSD1306_SWITCHCAPVCC, oledAddr, true, false)) {
    hasOled = true;
    display.cp437(true);
    showMessage("Display OK", "Looking for", "ADXL345...");
    Serial.printf("    OLED found at 0x%02X: the screen should say 'Display OK'\n", oledAddr);
  } else {
    Serial.println("    no OLED: running without a screen (data still goes to the Serial Monitor)");
  }

  // [4] ADXL345: keep retrying so a loose jumper can be fixed without a reset
  Serial.println("[4] Looking for the ADXL345 at 0x53 / 0x1D...");
  while ((adxlAddr = findAdxl()) == 0) {
    Serial.println("    ADXL345 not found (no DEVID 0xE5 at 0x53 or 0x1D). Retrying...");
    showMessage("ADXL345 not found", "Check SDA/SCL wiring", "CS->3V3, SDO->GND");
    delay(1000);
  }
  Serial.printf("    ADXL345 answered at 0x%02X\n", adxlAddr);

  // [5] Configure the accelerometer
  Serial.println("[5] Configuring ADXL345...");
  while (!adxlInit()) {
    Serial.println("    config write failed. Retrying...");
    showMessage("ADXL345 init failed", "Retrying...", "");
    delay(1000);
  }
  delay(20);  // first samples ready ~11 ms after enabling measurement at 100 Hz

  // Seed the filter with a real reading so it doesn't ramp up from zero.
  adxlReadG(ax, ay, az);
  updateTilt();

  // [6] Stepper. Let the filter settle for half a second first, so the
  //     starting angle (the motor's zero) is a clean reading.
  Serial.println("[6] Setting up the stepper (DRV8825)...");
  for (int i = 0; i < 50; i++) {
    float x, y, z;
    if (adxlReadG(x, y, z)) {
      ax += FILTER_ALPHA * (x - ax);
      ay += FILTER_ALPHA * (y - ay);
      az += FILTER_ALPHA * (z - az);
    }
    delay(SAMPLE_MS);
  }
  updateTilt();
  lastRollDeg = rollContinuous = rollSmoothed = rollHeld = rollZero = rollDeg;

  engine.init();
  stepper = engine.stepperConnectToPin(PIN_STEP);
  if (stepper == nullptr) {
    while (true) {
      Serial.println("    stepper setup failed: GPIO25 could not be used as a STEP pin");
      showMessage("Stepper setup failed", "STEP pin GPIO25", "");
      delay(2000);
    }
  }
  stepper->setDirectionPin(PIN_DIR);
  stepper->setEnablePin(PIN_EN, true);      // true = LOW enables the driver
  stepper->setSpeedInHz((uint32_t)(MAX_SPEED_DEG_S * STEPS_PER_DEG));
  stepper->setAcceleration((int32_t)(MAX_ACCEL_DEG_S2 * STEPS_PER_DEG));
  stepper->setCurrentPosition(0);
  if (QUIET_WHEN_IDLE) {
    // Coils on just before a move, off IDLE_OFF_MS after the last step.
    stepper->setDelayToEnable(2000);        // microseconds of settle time before the first step
    stepper->setDelayToDisable(IDLE_OFF_MS);
    stepper->setAutoEnable(true);
  } else {
    stepper->setAutoEnable(false);          // stay powered so the shaft holds its position
    stepper->enableOutputs();
  }
  Serial.printf("    ready: zero is R = %.1f deg, gain %.1fx, %.2f microsteps per degree\n",
                rollZero, MOTOR_GAIN, STEPS_PER_DEG);

  Serial.println("[7] Running. Live data follows:");
}

void loop() {
  static uint32_t lastSample = 0, lastDraw = 0;
  uint32_t now = millis();

  if (now - lastSample >= SAMPLE_MS) {
    lastSample = now;
    float x, y, z;
    if (adxlReadG(x, y, z)) {
      // Simple low-pass filter to calm the noise
      ax += FILTER_ALPHA * (x - ax);
      ay += FILTER_ALPHA * (y - ay);
      az += FILTER_ALPHA * (z - az);
      updateTilt();
      updateMotorTarget();
    } else {
      readErrors++;
    }
  }

  if (now - lastDraw >= DISPLAY_MS) {
    lastDraw = now;
    drawScreen();
    // "name:value" format works with Arduino IDE's Serial Plotter.
    // motor = shaft angle in degrees; it should mirror roll, MOTOR_GAIN times larger.
    Serial.printf("pitch:%.2f,roll:%.2f,ax:%.3f,ay:%.3f,az:%.3f,errors:%lu,motor:%.2f,hold:%d\n",
                  pitchDeg, rollDeg, ax, ay, az, (unsigned long)readErrors,
                  motorAngleDeg(), motorHolding ? 1 : 0);
  }

  handleSerialCommands();
}
