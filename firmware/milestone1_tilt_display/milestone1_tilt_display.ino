/*
  Milestone 1: ESP32 + ADXL345 + SSD1315 OLED over I2C
  -----------------------------------------------------
  Reads the ADXL345 accelerometer, works out tilt (pitch and roll) from the
  direction of gravity, and shows it on a 128x64 SSD1315 OLED along with a
  little bubble level.

  Wiring (ESP32 dev board, everything on one I2C bus):
    ESP32 3V3     -> ADXL345 VCC, OLED VCC
    ESP32 GND     -> ADXL345 GND, OLED GND
    ESP32 GPIO21  -> ADXL345 SDA, OLED SDA   (I2C data)
    ESP32 GPIO22  -> ADXL345 SCL, OLED SCL   (I2C clock)
    ADXL345 CS    -> 3V3   selects I2C mode (skip if your breakout already ties it high)
    ADXL345 SDO   -> GND   I2C address 0x53 (tie to 3V3 instead for 0x1D)

  Arduino IDE setup:
    Board package : "esp32 by Espressif Systems" -> Board: "ESP32 Dev Module"
    Library       : "Adafruit SSD1306" (also installs Adafruit GFX + BusIO)
    Serial Monitor: 115200 baud. The I2C scan at startup tells you what it found.

  The SSD1315 uses the same command set as the SSD1306, which is why the
  Adafruit SSD1306 library drives it without changes.
*/

#include <Wire.h>
#include <math.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

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

// Third and fourth arguments keep the bus at 400 kHz before and after each screen update.
// (By default the library drops the bus to 100 kHz after drawing.)
Adafruit_SSD1306 display(SCREEN_W, SCREEN_H, &Wire, -1, I2C_HZ, I2C_HZ);

uint8_t adxlAddr = 0;   // filled in by findAdxl()
uint8_t oledAddr = 0;

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

// ---------------- Display ----------------
void showMessage(const char *line1, const char *line2, const char *line3) {
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
// Print every device that answers on the bus. Expect 0x3C (OLED) and 0x53 (ADXL345).
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
  Serial.println("\n=== ESP32 + ADXL345 + SSD1315 tilt display ===");

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

  // [3] OLED: keep rescanning instead of halting silently
  Serial.println("[3] Looking for the OLED at 0x3C / 0x3D...");
  while (true) {
    if (i2cPresent(0x3C))      { oledAddr = 0x3C; break; }
    if (i2cPresent(0x3D))      { oledAddr = 0x3D; break; }
    Serial.println("    OLED not found. Rescanning in 2 s:");
    delay(2000);
    scanBus();
  }
  Serial.printf("    OLED answered at 0x%02X, initializing...\n", oledAddr);

  // periphBegin=false: we already started Wire on our pins above.
  if (!display.begin(SSD1306_SWITCHCAPVCC, oledAddr, true, false)) {
    while (true) {
      Serial.println("    display.begin() failed (could not allocate the screen buffer)");
      delay(2000);
    }
  }
  display.cp437(true);
  showMessage("Display OK", "Looking for", "ADXL345...");
  Serial.println("    OLED ready: the screen should say 'Display OK'");

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
  Serial.println("[6] Running. Live data follows:");
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
    } else {
      readErrors++;
    }
  }

  if (now - lastDraw >= DISPLAY_MS) {
    lastDraw = now;
    drawScreen();
    // "name:value" format works with Arduino IDE's Serial Plotter
    Serial.printf("pitch:%.2f,roll:%.2f,ax:%.3f,ay:%.3f,az:%.3f,errors:%lu\n",
                  pitchDeg, rollDeg, ax, ay, az, (unsigned long)readErrors);
  }
}
