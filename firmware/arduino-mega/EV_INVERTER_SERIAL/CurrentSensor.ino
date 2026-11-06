

const int CT_VOUT_PIN = A0;  // สาย Vout ต่อเข้าช่องแอนะล็อก A0
const int CT_REF_PIN  = A1;  // สาย Ref ต่อเข้าช่องแอนะล็อก A1

// Sensitivity ของรุ่น 15A คือ 0.04167 โวลต์ ต่อ 1 แอมป์ (Datasheet ยึดตามพิกัด 1 Turn)
const float CT_SENSITIVITY = 0.04167;

const int CT_PAIRS = 100;

const float ADC_TO_VOLT = 5.0 / 1023.0;

const int CT_ZERO_PAIRS = 1000;

// ค่าที่วัดได้ล่าสุด — main tab อ่านไปใช้ต่อ
float ctIdc   = 0.0;
float ctIpp   = 0.0;   // ระลอกคลื่นแปลงเป็นกระแส (A)
float ctVdiff = 0.0;   // Vout - Vref (V)
float ctVpp   = 0.0;   // peak-to-peak ที่ขา A0 (V)
float ctVref  = 0.0;   // แรงดันเฉลี่ยขา A1 (V)
float ctVout  = 0.0;   // แรงดันเฉลี่ยขา A0 (V)

float ctZeroVdiff = 0.0;
float ctZeroA     = 0.0;   // ตัวเดียวกันแปลงเป็นหน่วย A — main tab เอาไปรายงานใน ACK,ZERO

void setupCurrentSensor() {
  pinMode(CT_VOUT_PIN, INPUT);
  pinMode(CT_REF_PIN, INPUT);
}

void measureCurrent() {
  int32_t sumDiff = 0;
  uint32_t sumVout = 0, sumRef = 0;
  int16_t dmin = 32767, dmax = -32768;

  for (int i = 0; i < CT_PAIRS; i++) {
    // อ่านทิ้ง 1 ครั้งทุกครั้งที่สลับช่อง — ค่าแรกหลังสลับ mux ยังไม่นิ่ง
    analogRead(CT_VOUT_PIN);
    int16_t a0 = analogRead(CT_VOUT_PIN);

    analogRead(CT_REF_PIN);
    int16_t a1 = analogRead(CT_REF_PIN);

    int16_t d = a0 - a1;          // ลบกันทันที ตอน noise ยังเป็นค่าเดียวกันอยู่
    sumDiff += d;
    sumVout += a0;
    sumRef  += a1;
    if (d < dmin) dmin = d;
    if (d > dmax) dmax = d;
  }

  float meanDiff = (float)sumDiff / CT_PAIRS;

  ctVout  = ((float)sumVout / CT_PAIRS) * ADC_TO_VOLT;
  ctVref  = ((float)sumRef  / CT_PAIRS) * ADC_TO_VOLT;
  ctVdiff = meanDiff * ADC_TO_VOLT;
  ctVpp   = (float)(dmax - dmin) * ADC_TO_VOLT;   // ระลอกของ "ผลต่าง" ไม่ใช่ของ A0 เดี่ยวๆ

  ctIdc = (ctVdiff - ctZeroVdiff) / CT_SENSITIVITY;
  ctIpp = ctVpp / CT_SENSITIVITY;
}

void zeroCurrentSensor() {
  int32_t sumDiff = 0;
  for (int i = 0; i < CT_ZERO_PAIRS; i++) {

    wdt_reset();

    analogRead(CT_VOUT_PIN);
    int16_t a0 = analogRead(CT_VOUT_PIN);

    analogRead(CT_REF_PIN);
    int16_t a1 = analogRead(CT_REF_PIN);

    sumDiff += (int16_t)(a0 - a1);
  }
  ctZeroVdiff = ((float)sumDiff / CT_ZERO_PAIRS) * ADC_TO_VOLT;
  ctZeroA     = ctZeroVdiff / CT_SENSITIVITY;
}
