/*
 * EV4 Vehicle Bus - Test Sender
 *
 * Sends simulated EV4_Vehicle_Bus.dbc frames so the dash display can be
 * verified without the real vehicle bus.
 *
 * Serial commands (115200 baud):
 *   auto             - return to automatic tick-based simulation
 *   apps <0-100>     - APPS throttle percent
 *   torque <Nm>      - torque command (can be negative)
 *   power <kW>       - power
 *   lv <volts>       - LV voltage e.g. 12.4
 *   soc <0-100>      - battery SOC percent
 *   mode <0-5>       - drive mode: 0=STBY 1=DRIVE 2=REGEN 3=ENDUR 4=SPORT 5=LIMP
 *   fault <0|1>      - ECU/MC fault active
 *   inv1fault <0|1>  - inverter 1 MC fault active
 *   inv2fault <0|1>  - inverter 2 MC fault active
 *   init <0|1>       - init finished
 *   r2d <0|1>        - ready to drive
 *   pre <0|1>        - precharge complete
 *   regen <0|1>      - regen enabled
 *   bps <raw>        - brake pressure sensor raw value
 *   prog1 <0|1>      - program button 1
 *   prog2 <0|1>      - program button 2
 *   bms_i <amps>     - BMS current
 *   maxcellv <volts> - max cell voltage e.g. 4.08
 *   mincellv <volts> - min cell voltage e.g. 3.72
 *   maxcellt <C>     - max cell temp
 *   mincellt <C>     - min cell temp
 *   powerlim <kW>    - BMS power limit
 *   water <t1> <t2> <t3> - water temps in C
 *   brake <0|1>      - brake pressed
 *   accelx <m/s^2>   - IMU X acceleration
 *   accely <m/s^2>   - IMU Y acceleration
 *   accelz <m/s^2>   - IMU Z acceleration
 *   gyrox <rad/s>    - IMU X gyro
 *   gyroy <rad/s>    - IMU Y gyro
 *   gyroz <rad/s>    - IMU Z gyro
 *   show             - print current values
 *   help             - print this list
 *
 * MCP2515 wiring (HSPI):
 *   MCP2515 SCK -> GPIO 14
 *   MCP2515 SI  -> GPIO 13
 *   MCP2515 SO  -> GPIO 12
 *   MCP2515 CS  -> GPIO 15
 *   MCP2515 INT -> GPIO  4
 */

#include <SPI.h>
#include <mcp_can.h>

#define CAN_CS    15
#define CAN_INT    4
#define HSPI_SCK  14
#define HSPI_MISO 12
#define HSPI_MOSI 13

SPIClass hspi(HSPI);
MCP_CAN CAN(&hspi, CAN_CS);

uint32_t lastSendMs = 0;
uint16_t tick = 0;

// ── Manual override state ────────────────────────────────────
bool manualMode = true;

struct ManualState {
  uint8_t  appsPct      = 50;
  int16_t  torqueCmd    = 100;
  uint16_t powerKw      = 40;
  uint16_t lvAdc        = 124;  // x0.1 = volts
  uint8_t  battSoc      = 80;
  uint8_t  driveMode    = 1;
  bool     fault        = false;
  bool     inv1Fault    = false;
  bool     inv2Fault    = false;
  bool     initFinished = true;
  bool     r2d          = true;
  bool     precharge    = true;
  bool     regen        = false;
  bool     brake        = false;
  bool     prog1        = false;
  bool     prog2        = false;
  uint16_t bpsRaw       = 300;
  float    bmsCurrent   = 50.0f;
  float    maxCellV     = 4.08f;
  float    minCellV     = 3.72f;
  float    maxCellTempC = 36.0f;
  float    minCellTempC = 30.0f;
  uint8_t  powerLimKw   = 80;
  uint8_t  water1C      = 34;
  uint8_t  water2C      = 36;
  uint8_t  water3C      = 38;
  float    accelX       = 0.0f;
  float    accelY       = 0.0f;
  float    accelZ       = 9.81f;
  float    gyroX        = 0.0f;
  float    gyroY        = 0.0f;
  float    gyroZ        = 0.0f;
};

ManualState manual;
String serialLine;

// ── CAN encode helpers ───────────────────────────────────────
void putU16(uint8_t *buf, uint8_t byteIndex, uint16_t value) {
  buf[byteIndex]     = (uint8_t)(value & 0xFF);
  buf[byteIndex + 1] = (uint8_t)(value >> 8);
}

void putS16(uint8_t *buf, uint8_t byteIndex, int16_t value) {
  putU16(buf, byteIndex, (uint16_t)value);
}

void putFloat(uint8_t *buf, uint8_t byteIndex, float value) {
  memcpy(buf + byteIndex, &value, 4);
}

void setBit(uint8_t *buf, uint8_t bit, bool value) {
  uint8_t byteIndex = bit / 8;
  uint8_t mask = 1 << (bit % 8);
  if (value) buf[byteIndex] |= mask;
  else        buf[byteIndex] &= ~mask;
}

uint8_t encodeBmsSoc(float pct)     { return (uint8_t)(constrain(pct,   0,  100) / 0.392156863f + 0.5f); }
uint8_t encodeBmsCurrent(float a)   { return (uint8_t)(constrain(a,     0,  200) / 0.78125f     + 0.5f); }
uint8_t encodeCellVolt(float v)     { return (uint8_t)(constrain(v,     0,    5) / 0.019607843f + 0.5f); }
uint8_t encodeCellTemp(float t)     { return (uint8_t)(constrain(t,     0,  150) / 0.588235294f + 0.5f); }

// ── Serial helpers ───────────────────────────────────────────
float clampF(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

void printHelp() {
  Serial.println();
  Serial.println("Commands:");
  Serial.println("  auto             resume tick-based simulation");
  Serial.println("  apps <0-100>     APPS percent");
  Serial.println("  torque <Nm>      torque command");
  Serial.println("  power <kW>       power");
  Serial.println("  lv <volts>       LV voltage e.g. 12.4");
  Serial.println("  soc <0-100>      battery SOC");
  Serial.println("  mode <0-5>       0=STBY 1=DRIVE 2=REGEN 3=ENDUR 4=SPORT 5=LIMP");
  Serial.println("  fault <0|1>      fault active");
  Serial.println("  inv1fault <0|1>  inverter 1 MC fault active");
  Serial.println("  inv2fault <0|1>  inverter 2 MC fault active");
  Serial.println("  init <0|1>       init finished");
  Serial.println("  r2d <0|1>        ready to drive");
  Serial.println("  pre <0|1>        precharge complete");
  Serial.println("  regen <0|1>      regen enabled");
  Serial.println("  brake <0|1>      brake pressed");
  Serial.println("  bps <raw>        brake pressure sensor raw value");
  Serial.println("  prog1 <0|1>      program button 1");
  Serial.println("  prog2 <0|1>      program button 2");
  Serial.println("  bms_i <amps>     BMS current");
  Serial.println("  maxcellv <V>     max cell voltage");
  Serial.println("  mincellv <V>     min cell voltage");
  Serial.println("  maxcellt <C>     max cell temp");
  Serial.println("  mincellt <C>     min cell temp");
  Serial.println("  powerlim <kW>    BMS power limit");
  Serial.println("  water <t1> <t2> <t3>  water temps in C");
  Serial.println("  accelx <m/s^2>   IMU X acceleration");
  Serial.println("  accely <m/s^2>   IMU Y acceleration");
  Serial.println("  accelz <m/s^2>   IMU Z acceleration");
  Serial.println("  gyrox <rad/s>    IMU X gyro");
  Serial.println("  gyroy <rad/s>    IMU Y gyro");
  Serial.println("  gyroz <rad/s>    IMU Z gyro");
  Serial.println("  show             print current values");
  Serial.println("  help             print this list");
  Serial.println();
}

void printState() {
  Serial.print("mode=");       Serial.print(manual.driveMode);
  Serial.print(" apps=");      Serial.print(manual.appsPct);
  Serial.print("% torque=");   Serial.print(manual.torqueCmd);
  Serial.print("Nm power=");   Serial.print(manual.powerKw);
  Serial.print("kW lv=");      Serial.print(manual.lvAdc * 0.1f, 1);
  Serial.print("V soc=");      Serial.print(manual.battSoc);
  Serial.print("% fault=");    Serial.print(manual.fault);
  Serial.print(" inv1=");      Serial.print(manual.inv1Fault);
  Serial.print(" inv2=");      Serial.print(manual.inv2Fault);
  Serial.print(" init=");      Serial.print(manual.initFinished);
  Serial.print(" r2d=");       Serial.print(manual.r2d);
  Serial.print(" pre=");       Serial.print(manual.precharge);
  Serial.print(" regen=");     Serial.print(manual.regen);
  Serial.print(" brake=");     Serial.print(manual.brake);
  Serial.print(" bps=");       Serial.print(manual.bpsRaw);
  Serial.print(" prog1=");     Serial.print(manual.prog1);
  Serial.print(" prog2=");     Serial.println(manual.prog2);
  Serial.print("bms_i=");      Serial.print(manual.bmsCurrent, 1);
  Serial.print("A cellV=");    Serial.print(manual.minCellV, 2);
  Serial.print("/");           Serial.print(manual.maxCellV, 2);
  Serial.print("V cellT=");    Serial.print(manual.minCellTempC, 0);
  Serial.print("/");           Serial.print(manual.maxCellTempC, 0);
  Serial.print("C water=");    Serial.print(manual.water1C);
  Serial.print("/");           Serial.print(manual.water2C);
  Serial.print("/");           Serial.print(manual.water3C);
  Serial.print("C plim=");     Serial.print(manual.powerLimKw);
  Serial.println("kW");
  Serial.print("accel=");      Serial.print(manual.accelX, 3);
  Serial.print("/");           Serial.print(manual.accelY, 3);
  Serial.print("/");           Serial.print(manual.accelZ, 3);
  Serial.print(" gyro=");      Serial.print(manual.gyroX, 3);
  Serial.print("/");           Serial.print(manual.gyroY, 3);
  Serial.print("/");           Serial.println(manual.gyroZ, 3);
}

void handleCommand(String line) {
  line.trim();
  if (line.length() == 0) return;

  int si = line.indexOf(' ');
  String cmd = (si >= 0) ? line.substring(0, si) : line;
  String rest = (si >= 0) ? line.substring(si + 1) : "";
  cmd.toLowerCase();
  rest.trim();

  float arg1 = rest.toFloat();

  if (cmd == "help" || cmd == "?") {
    printHelp();
    return;
  }
  if (cmd == "show") { printState(); return; }

  if (cmd == "auto") {
    manualMode = false;
    Serial.println("Auto simulation resumed.");
    return;
  }

  // All other commands switch to manual mode
  manualMode = true;

  if      (cmd == "apps")     { manual.appsPct    = (uint8_t)clampF(arg1, 0, 100); }
  else if (cmd == "torque")   { manual.torqueCmd  = (int16_t)clampF(arg1, -32768, 32767); }
  else if (cmd == "power")    { manual.powerKw    = (uint16_t)clampF(arg1, 0, 65535); }
  else if (cmd == "lv")       { manual.lvAdc      = (uint16_t)(clampF(arg1, 0, 99) * 10 + 0.5f); }
  else if (cmd == "soc")      { manual.battSoc    = (uint8_t)clampF(arg1, 0, 100); }
  else if (cmd == "mode")     { manual.driveMode  = (uint8_t)clampF(arg1, 0, 5); }
  else if (cmd == "fault")    {
    manual.fault = (rest.toInt() != 0);
    manual.inv1Fault = manual.fault;
    manual.inv2Fault = manual.fault;
  }
  else if (cmd == "inv1fault") { manual.inv1Fault = (rest.toInt() != 0); }
  else if (cmd == "inv2fault") { manual.inv2Fault = (rest.toInt() != 0); }
  else if (cmd == "init")     { manual.initFinished = (rest.toInt() != 0); }
  else if (cmd == "r2d")      { manual.r2d        = (rest.toInt() != 0); }
  else if (cmd == "pre")      { manual.precharge  = (rest.toInt() != 0); }
  else if (cmd == "regen")    { manual.regen      = (rest.toInt() != 0); }
  else if (cmd == "brake")    { manual.brake      = (rest.toInt() != 0); }
  else if (cmd == "bps")      { manual.bpsRaw     = (uint16_t)clampF(arg1, 0, 65535); }
  else if (cmd == "prog1")    { manual.prog1      = (rest.toInt() != 0); }
  else if (cmd == "prog2")    { manual.prog2      = (rest.toInt() != 0); }
  else if (cmd == "bms_i")    { manual.bmsCurrent   = clampF(arg1, 0, 200); }
  else if (cmd == "maxcellv") { manual.maxCellV     = clampF(arg1, 0, 5); }
  else if (cmd == "mincellv") { manual.minCellV     = clampF(arg1, 0, 5); }
  else if (cmd == "maxcellt") { manual.maxCellTempC = clampF(arg1, 0, 150); }
  else if (cmd == "mincellt") { manual.minCellTempC = clampF(arg1, 0, 150); }
  else if (cmd == "powerlim") { manual.powerLimKw   = (uint8_t)clampF(arg1, 0, 255); }
  else if (cmd == "accelx")   { manual.accelX = arg1; }
  else if (cmd == "accely")   { manual.accelY = arg1; }
  else if (cmd == "accelz")   { manual.accelZ = arg1; }
  else if (cmd == "gyrox")    { manual.gyroX  = arg1; }
  else if (cmd == "gyroy")    { manual.gyroY  = arg1; }
  else if (cmd == "gyroz")    { manual.gyroZ  = arg1; }
  else if (cmd == "water") {
    int s1 = rest.indexOf(' ');
    if (s1 >= 0) {
      String r2 = rest.substring(s1 + 1);
      int s2 = r2.indexOf(' ');
      manual.water1C = (uint8_t)clampF(rest.toFloat(), 0, 255);
      manual.water2C = (uint8_t)clampF(r2.toFloat(), 0, 255);
      if (s2 >= 0) manual.water3C = (uint8_t)clampF(r2.substring(s2 + 1).toFloat(), 0, 255);
    }
  }
  else {
    Serial.print("Unknown command: "); Serial.println(cmd);
    printHelp();
    return;
  }

  printState();
}

void readSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (serialLine.length() > 0) {
        handleCommand(serialLine);
        serialLine = "";
      }
    } else {
      serialLine += c;
      if (serialLine.length() > 80) { serialLine = ""; Serial.println("Input too long."); }
    }
  }
}

// ── Frame sending ────────────────────────────────────────────
void sendFrame(uint16_t id, uint8_t *buf) {
  CAN.sendMsgBuf(id, 0, 8, buf);
}

void sendAutoFrames() {
  uint8_t buf[8] = {0};

  // t in seconds (tick fires every 250 ms)
  float t = tick * 0.25f;

  // ── Throttle: sawtooth 0→100% over 20 s, brief lift at top ──
  float sawPos  = fmodf(t, 20.0f) / 20.0f;           // 0.0–1.0
  uint8_t appsPct = (sawPos < 0.85f)
                    ? (uint8_t)(sawPos / 0.85f * 100.0f)
                    : (uint8_t)((1.0f - sawPos) / 0.15f * 15.0f);

  bool regen    = (appsPct < 8);
  int16_t torqueCmd = regen ? (int16_t)(-40)
                             : (int16_t)(appsPct * 3);
  uint16_t powerKw  = (uint16_t)(appsPct * 80UL / 100UL);

  // ── SOC: drains 100→20% over ~80 s then resets ──────────────
  uint8_t battSoc = (uint8_t)(100.0f - fmodf(t, 80.0f) / 80.0f * 80.0f);

  // ── LV voltage: 12.0–13.5 V gentle oscillation ───────────────
  uint16_t lvAdc = (uint16_t)(125.0f + 8.0f * sinf(t * 0.15f)); // x0.1 V

  // ── Drive mode: cycles through all 6 every 15 s ──────────────
  uint8_t driveMode = (uint8_t)(fmodf(t, 90.0f) / 15.0f);

  // ── State flags ───────────────────────────────────────────────
  bool precharge = true;
  bool r2d       = (fmodf(t, 40.0f) >= 3.0f);   // 3-s gap at start of each lap
  bool initDone  = (fmodf(t, 40.0f) >= 1.5f);

  // ── Brief fault pulse every 30 s for 2 s ─────────────────────
  bool demoFault = (fmodf(t, 30.0f) < 2.0f);

  // ── BPS / brake: spikes when throttle lifts ──────────────────
  bool brakeOn  = regen && (appsPct < 4);
  uint16_t bpsRaw = brakeOn ? 650 : (uint16_t)(180.0f + 40.0f * sinf(t * 0.4f));

  // ── Water temps: rise with load, baseline oscillation ────────
  uint8_t water1C = (uint8_t)(30.0f + appsPct * 0.18f + 4.0f * sinf(t * 0.07f));
  uint8_t water2C = water1C + 3;
  uint8_t water3C = water1C + 6;

  // ── Cell voltages: track SOC ──────────────────────────────────
  float maxCellV  = 3.60f + (battSoc / 100.0f) * 0.58f;   // 3.60–4.18 V
  float minCellV  = maxCellV - 0.12f - 0.04f * sinf(t * 0.2f);

  // ── Cell temps: load-dependent ────────────────────────────────
  float maxCellT  = 25.0f + appsPct * 0.22f + 4.0f * sinf(t * 0.11f);
  float minCellT  = maxCellT - 7.0f;

  // ── BMS current/limit ─────────────────────────────────────────
  float bmsCurrent = 8.0f + appsPct * 1.6f;
  uint8_t powerLim = (maxCellT > 42.0f) ? 60 : 80;

  // ── IMU: simulated lap cornering ─────────────────────────────
  float accelX = 3.5f * sinf(t * 0.25f);              // lateral ±3.5 m/s²
  float accelY = (appsPct / 100.0f) * 6.0f            // longitudinal
                 - (brakeOn ? 4.0f : 0.0f);
  float accelZ = 9.81f + 0.4f * sinf(t * 0.55f);
  float gyroX  = 0.08f * sinf(t * 0.9f);
  float gyroY  = 0.08f * cosf(t * 0.9f);
  float gyroZ  = 0.35f * sinf(t * 0.25f);             // yaw rate with accelX

  // ── Send frames ───────────────────────────────────────────────

  // ID 0x2 ECU_FAULTS
  memset(buf, 0, 8);
  setBit(buf, 4, demoFault);
  setBit(buf, 7, demoFault);
  sendFrame(0x2, buf);

  // ID 0x3 MC_FAULTS
  memset(buf, 0, 8);
  setBit(buf, 2, demoFault);
  setBit(buf, 25, demoFault);
  setBit(buf, 34, demoFault);
  setBit(buf, 57, demoFault);
  sendFrame(0x3, buf);

  // ID 0x4 APPS_Info
  memset(buf, 0, 8);
  putU16(buf, 0, 800 + appsPct * 10);
  putU16(buf, 2, 820 + appsPct * 10);
  buf[4] = appsPct;
  putS16(buf, 5, torqueCmd);
  sendFrame(0x4, buf);

  // ID 0x5 Sensors_Info
  memset(buf, 0, 8);
  putU16(buf, 0, bpsRaw);
  buf[2] = water1C;
  buf[3] = water2C;
  buf[4] = water3C;
  setBit(buf, 40, r2d);
  setBit(buf, 41, false);
  setBit(buf, 42, false);
  setBit(buf, 43, brakeOn);
  sendFrame(0x5, buf);

  // ID 0x6 Internal_States
  memset(buf, 0, 8);
  putU16(buf, 0, powerKw);
  putU16(buf, 2, lvAdc);
  buf[4] = battSoc;
  setBit(buf, 40, initDone);
  setBit(buf, 41, precharge);
  setBit(buf, 42, r2d);
  buf[5] |= (driveMode & 0x07) << 3;
  setBit(buf, 46, regen);
  sendFrame(0x6, buf);

  // ID 0x7 BMS_Info
  memset(buf, 0, 8);
  buf[0] = encodeBmsSoc((float)battSoc);
  buf[1] = encodeBmsCurrent(bmsCurrent);
  buf[2] = encodeCellVolt(maxCellV);
  buf[3] = encodeCellTemp(maxCellT);
  buf[4] = encodeCellVolt(minCellV);
  buf[5] = encodeCellTemp(minCellT);
  buf[6] = powerLim;
  sendFrame(0x7, buf);

  // ID 0x8 IMU_Z  (Accel_Z bytes 0-3, Gyro_Z bytes 4-7)
  memset(buf, 0, 8);
  putFloat(buf, 0, accelZ);
  putFloat(buf, 4, gyroZ);
  sendFrame(0x8, buf);

  // ID 0x9 IMU_Gyro_XY  (Gyro_X bytes 0-3, Gyro_Y bytes 4-7)
  memset(buf, 0, 8);
  putFloat(buf, 0, gyroX);
  putFloat(buf, 4, gyroY);
  sendFrame(0x9, buf);

  // ID 0xA IMU_Accel_XY  (Accel_X bytes 0-3, Accel_Y bytes 4-7)
  memset(buf, 0, 8);
  putFloat(buf, 0, accelX);
  putFloat(buf, 4, accelY);
  sendFrame(0xA, buf);

  tick++;
}

void sendManualFrames() {
  uint8_t buf[8] = {0};

  // ID 0x2 ECU_FAULTS
  memset(buf, 0, 8);
  setBit(buf, 4, manual.fault);
  setBit(buf, 7, manual.fault);
  setBit(buf, 9, manual.fault);
  sendFrame(0x2, buf);

  // ID 0x3 MC_FAULTS
  memset(buf, 0, 8);
  setBit(buf, 2, manual.inv1Fault);
  setBit(buf, 25, manual.inv1Fault);
  setBit(buf, 34, manual.inv2Fault);
  setBit(buf, 57, manual.inv2Fault);
  sendFrame(0x3, buf);

  // ID 0x4 APPS_Info
  memset(buf, 0, 8);
  putU16(buf, 0, 800 + manual.appsPct * 10);
  putU16(buf, 2, 820 + manual.appsPct * 10);
  buf[4] = manual.appsPct;
  putS16(buf, 5, manual.torqueCmd);
  sendFrame(0x4, buf);

  // ID 0x5 Sensors_Info
  memset(buf, 0, 8);
  putU16(buf, 0, manual.bpsRaw);
  buf[2] = manual.water1C;
  buf[3] = manual.water2C;
  buf[4] = manual.water3C;
  setBit(buf, 40, manual.r2d);
  setBit(buf, 41, manual.prog1);
  setBit(buf, 42, manual.prog2);
  setBit(buf, 43, manual.brake);
  sendFrame(0x5, buf);

  // ID 0x6 Internal_States
  memset(buf, 0, 8);
  putU16(buf, 0, manual.powerKw);
  putU16(buf, 2, manual.lvAdc);
  buf[4] = manual.battSoc;
  setBit(buf, 40, manual.initFinished);
  setBit(buf, 41, manual.precharge);
  setBit(buf, 42, manual.r2d);
  buf[5] |= (manual.driveMode & 0x07) << 3;
  setBit(buf, 46, manual.regen);
  sendFrame(0x6, buf);

  // ID 0x7 BMS_Info
  memset(buf, 0, 8);
  buf[0] = encodeBmsSoc((float)manual.battSoc);
  buf[1] = encodeBmsCurrent(manual.bmsCurrent);
  buf[2] = encodeCellVolt(manual.maxCellV);
  buf[3] = encodeCellTemp(manual.maxCellTempC);
  buf[4] = encodeCellVolt(manual.minCellV);
  buf[5] = encodeCellTemp(manual.minCellTempC);
  buf[6] = manual.powerLimKw;
  sendFrame(0x7, buf);

  // ID 0x8 IMU_Z  (Accel_Z bytes 0-3, Gyro_Z bytes 4-7)
  memset(buf, 0, 8);
  putFloat(buf, 0, manual.accelZ);
  putFloat(buf, 4, manual.gyroZ);
  sendFrame(0x8, buf);

  // ID 0x9 IMU_Gyro_XY  (Gyro_X bytes 0-3, Gyro_Y bytes 4-7)
  memset(buf, 0, 8);
  putFloat(buf, 0, manual.gyroX);
  putFloat(buf, 4, manual.gyroY);
  sendFrame(0x9, buf);

  // ID 0xA IMU_Accel_XY  (Accel_X bytes 0-3, Accel_Y bytes 4-7)
  memset(buf, 0, 8);
  putFloat(buf, 0, manual.accelX);
  putFloat(buf, 4, manual.accelY);
  sendFrame(0xA, buf);
}

// ── Setup / loop ─────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("[EV4 Vehicle Bus Test Sender] Boot");

  hspi.begin(HSPI_SCK, HSPI_MISO, HSPI_MOSI, CAN_CS);
  pinMode(CAN_INT, INPUT);

  while (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) != CAN_OK) {
    Serial.println("MCP2515 init failed, retrying...");
    delay(500);
  }

  CAN.setMode(MCP_NORMAL);
  Serial.println("MCP2515 ready @ 500 kbps");
  printHelp();
}

void loop() {
  readSerial();

  uint32_t now = millis();
  if (now - lastSendMs >= 250) {
    lastSendMs = now;
    if (manualMode) sendManualFrames();
    else            sendAutoFrames();
  }
}
