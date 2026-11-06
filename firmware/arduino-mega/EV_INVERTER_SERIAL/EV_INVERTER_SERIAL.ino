

#include <avr/wdt.h>

const int pinON = 22;
const int pinOFF = 26;

bool inverterRunning = false; // ตัวแปรสถานะระบบอินเวอร์เตอร์

const uint16_t pwmTop = 94;

const int duty = 47;
const int deadTime = 4;  //Dead-time 2.1% , 250 ns

const int onValueA = duty - (deadTime / 2);             // 45
const int onValueB = duty + (deadTime - deadTime / 2);  // 49

const long SERIAL_BAUD = 115200;

// ส่งค่ากระแสขึ้น Pi ทุกกี่มิลลิวินาที
const unsigned long TELEM_INTERVAL_MS = 100;

const unsigned long PING_TIMEOUT_MS = 800;

const float OC_LIMIT_A = 5.0;

float ocLimitA = OC_LIMIT_A;

extern float ctIdc, ctIpp, ctVdiff, ctVpp, ctVref, ctVout, ctZeroA;

// สถานะภายใน
bool startedBySerial = false;      // เปิดด้วยคำสั่งจาก Pi หรือด้วยปุ่มกดมือ

bool prevOnLevel  = HIGH;
bool prevOffLevel = HIGH;

bool ocLockout = false;

unsigned long lastPingMs = 0;
unsigned long lastTelemMs = 0;

String rxLine = "";

void setup() {

  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(2, OUTPUT);

  pinMode(pinON, INPUT_PULLUP);
  pinMode(pinOFF, INPUT_PULLUP);

  prevOnLevel  = digitalRead(pinON);
  prevOffLevel = digitalRead(pinOFF);

  // 3+4. ตั้งค่า Timer1/Timer3 ให้เริ่มนับพร้อมกันเป๊ะ
  setupSyncedTimers();

  // 5. สั่งหยุดการทำงานเอาต์พุตไว้ตั้งแต่เริ่มต้น
  stopInverter();

  setupCurrentSensor();
  Serial.begin(SERIAL_BAUD);
  rxLine.reserve(16);
  Serial.println(F("BOOT,EV_INVERTER_SERIAL"));

  wdt_enable(WDTO_2S);
}

void loop() {
  wdt_reset();
  handleSerial();
  handleButtons();
  measureCurrent();
  safetyCheck();
  sendTelemetry();
}

void handleButtons() {
  bool nowOn  = digitalRead(pinON);
  bool nowOff = digitalRead(pinOFF);

  // ---- สวิตช์เปิดระบบพิน 22: ทำงานเฉพาะขอบขาลง ----
  if (prevOnLevel == HIGH && nowOn == LOW) {
    if (ocLockout) {

      Serial.println(F("ERR,OC_LOCKOUT"));
    } else if (!inverterRunning) {
      startInverter();
      startedBySerial = false;   // เปิดด้วยมือ -> ไม่ต้องมี watchdog
      Serial.println(F("EVT,BTN_ON"));
    }
  }

  // ---- สวิตช์ปิดระบบพิน 26: ทำงานเฉพาะขอบขาลง ----
  if (prevOffLevel == HIGH && nowOff == LOW) {

    ocLockout = false;
    if (inverterRunning) {
      stopInverter();
      startedBySerial = false;
      Serial.println(F("EVT,BTN_OFF"));
    }
  }

  prevOnLevel  = nowOn;
  prevOffLevel = nowOff;
}

void handleSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (rxLine.length() > 0) {
        execCommand(rxLine);
        rxLine = "";
      }
    } else if (rxLine.length() < 12) {
      rxLine += c;
    }
  }
}

void execCommand(const String &cmd) {
  if (cmd == "ON") {
    lastPingMs = millis();
    if (!inverterRunning) {
      startInverter();
    }
    startedBySerial = true;
    Serial.println(F("ACK,ON"));
  }
  else if (cmd == "OFF") {
    if (inverterRunning) {
      stopInverter();
    }
    startedBySerial = false;

    ocLockout = false;
    Serial.println(F("ACK,OFF"));
  }
  else if (cmd == "PING") {
    lastPingMs = millis();
  }
  else if (cmd == "ZERO") {

    if (inverterRunning) {
      Serial.println(F("ERR,ZERO_WHILE_ON"));
    } else {
      zeroCurrentSensor();   // บล็อก ~420ms — inverter ดับอยู่ ไม่มีอะไรต้องรีบ
      Serial.print(F("ACK,ZERO,"));
      Serial.println(ctZeroA, 3);   // รายงาน offset เป็นหน่วย A ไว้ log
    }
  }
  else if (cmd == "OC?") {

    Serial.print(F("ACK,OC,"));
    Serial.println(ocLimitA, 2);
  }
  else if (cmd.startsWith("OC,")) {

    float v = cmd.substring(3).toFloat();
    if (v > 0.0) {
      ocLimitA = v;
      Serial.print(F("ACK,OC,"));
      Serial.println(ocLimitA, 2);
    } else {
      Serial.println(F("ERR,OC_RANGE"));
    }
  }
}

void safetyCheck() {
  if (!inverterRunning) return;

  if (ocLimitA > 0.0 && fabs(ctIdc) > ocLimitA) {
    stopInverter();
    startedBySerial = false;

    ocLockout = true;
    Serial.println(F("EVT,OC"));
    return;
  }

  // 2. Pi เงียบหายไป (ค้าง / สายหลุด / โดน kill) — เฉพาะกรณีที่ Pi เป็นคนสั่งเปิด
  if (startedBySerial && (millis() - lastPingMs > PING_TIMEOUT_MS)) {
    stopInverter();
    startedBySerial = false;
    Serial.println(F("EVT,WDT"));
  }
}

void sendTelemetry() {
  unsigned long now = millis();
  if (now - lastTelemMs < TELEM_INTERVAL_MS) return;
  lastTelemMs = now;

  Serial.print(F("D,"));
  Serial.print(inverterRunning ? 1 : 0);
  Serial.print(',');  Serial.print(ctIdc, 3);
  Serial.print(',');  Serial.print(ctIpp, 3);
  Serial.print(',');  Serial.print(ctVdiff, 4);
  Serial.print(',');  Serial.print(ctVpp, 4);
  Serial.print(',');  Serial.print(ctVref, 3);
  Serial.print(',');  Serial.println(ctVout, 3);
}

void setupSyncedTimers() {
  uint8_t oldSREG = SREG;
  cli();  // ห้าม interrupt มาแทรกระหว่างตั้งค่า

  // freeze + reset prescaler
  GTCCR = _BV(TSM) | _BV(PSRSYNC);

  TCCR1A = 0; TCCR1B = 0; TCCR1C = 0;
  TCCR3A = 0; TCCR3B = 0; TCCR3C = 0;

  // Phase and Frequency Correct PWM, TOP = ICR (Mode 8), No Prescaler (CS = 001)
  TCCR1B = _BV(WGM13) | _BV(CS10);
  TCCR3B = _BV(WGM33) | _BV(CS30);

  ICR1 = pwmTop;
  ICR3 = pwmTop;

  // ตั้ง OCR ไว้ล่วงหน้า (ยังไม่ต่อออกขา เพราะ TCCRnA = 0)
  OCR1A = onValueA;
  OCR1B = onValueB;
  OCR3A = onValueA;
  OCR3B = onValueB;

  // เคลียร์ตัวนับให้เป็นศูนย์พร้อมกัน
  TCNT1 = 0;
  TCNT3 = 0;

  // ปล่อยพร้อมกัน -> TCNT1 กับ TCNT3 ตรงกันทุก tick
  GTCCR = 0;

  SREG = oldSREG;
}

void startInverter() {
  uint8_t oldSREG = SREG;
  cli();

  OCR1A = onValueA;
  OCR1B = onValueB;
  OCR3A = onValueA;
  OCR3B = onValueB;

  TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(COM1B0);
  TCCR3A = _BV(COM3A1) | _BV(COM3B1) | _BV(COM3B0);

  SREG = oldSREG;

  inverterRunning = true;
}

// ฟังก์ชันสั่งตัดสัญญาณพัลส์ทุกช่องลงดินสนิท
void stopInverter() {
  uint8_t oldSREG = SREG;
  cli();

  TCCR1A = 0;   // ปลดขาออกจาก timer
  TCCR3A = 0;

  SREG = oldSREG;

  digitalWrite(11, LOW);
  digitalWrite(12, LOW);
  digitalWrite(5, LOW);
  digitalWrite(2, LOW);

  inverterRunning = false;
}
