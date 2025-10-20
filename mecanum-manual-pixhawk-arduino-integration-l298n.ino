// ===== Mecanum Rover (Arduino Leonardo) =====
// RC inputs via external interrupts: Y->D0, X->D1, W->D7
// L298N pins per wiring below
// Soft-start ramp + simple failsafe

#include <Arduino.h>
#include <math.h>

// ---------- User Tuning ----------
const int  DEADZONE_US_LOW  = 1470;
const int  DEADZONE_US_HIGH = 1530;

const bool INVERT_X = false;         // flip if strafe feels reversed
const bool INVERT_Y = false;         // flip if forward feels reversed
const bool INVERT_W = false;         // flip if rotation feels reversed
const float MAX_SCALE_MIX = 1.00f;   // 0..1 overall cap

// Per-wheel inversion (left side inverted)
#define INV_LF 1
#define INV_RF 0
#define INV_LR 1
#define INV_RR 0

// Failsafe & soft-start
const unsigned long STALE_US      = 100000UL; // >100ms since last edge => stale
const uint8_t       RAMP_STEP_PWM = 6;        // smaller = gentler ramp

// ---------- Motor pins ----------
const uint8_t LF_IN1 = 2,  LF_IN2 = 4,  LF_EN = 5;   // PWM
const uint8_t LR_IN1 = 6,  LR_IN2 = 11, LR_EN = 3;   // PWM (LR_IN2 moved from D7->D11)
const uint8_t RF_IN1 = 12, RF_IN2 = 13, RF_EN = 9;   // PWM
const uint8_t RR_IN1 = A0, RR_IN2 = A1, RR_EN = 10;  // PWM

// ---------- Motor struct & array ----------
struct Motor {
  uint8_t in1, in2, en, invert;
  int curr; // current PWM (for ramping)
};

Motor M[4] = {
  {LF_IN1, LF_IN2, LF_EN, INV_LF, 0}, // 0 LF
  {RF_IN1, RF_IN2, RF_EN, INV_RF, 0}, // 1 RF
  {LR_IN1, LR_IN2, LR_EN, INV_LR, 0}, // 2 LR
  {RR_IN1, RR_IN2, RR_EN, INV_RR, 0}  // 3 RR
};

// ---------- RC INPUT via interrupts (Leonardo) ----------
// Pins: Y->D0 (int#2), X->D1 (int#3), W->D7 (int#4)
const uint8_t PIN_Y_INT = 0;  // Y (pitch)
const uint8_t PIN_X_INT = 1;  // X (roll)
const uint8_t PIN_W_INT = 7;  // W (yaw/rotation)

volatile unsigned long yRise=0, yWidth=1500, yStamp=0;
volatile unsigned long xRise=0, xWidth=1500, xStamp=0;
volatile unsigned long wRise=0, wWidth=1500, wStamp=0;

void isrY(){
  if (digitalRead(PIN_Y_INT)) { yRise = micros(); }
  else { unsigned long now=micros(); yWidth = now - yRise; yStamp = now; }
}
void isrX(){
  if (digitalRead(PIN_X_INT)) { xRise = micros(); }
  else { unsigned long now=micros(); xWidth = now - xRise; xStamp = now; }
}
void isrW(){
  if (digitalRead(PIN_W_INT)) { wRise = micros(); }
  else { unsigned long now=micros(); wWidth = now - wRise; wStamp = now; }
}

void rcInputBegin(){
  pinMode(PIN_Y_INT, INPUT);
  pinMode(PIN_X_INT, INPUT);
  pinMode(PIN_W_INT, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_Y_INT), isrY, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_X_INT), isrX, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_W_INT), isrW, CHANGE);
}

// Fetch fresh µs; returns 0 if stale
unsigned long rcReadUs(char axis){
  noInterrupts();
  unsigned long w = (axis=='Y') ? yWidth : (axis=='X') ? xWidth : wWidth;
  unsigned long t = (axis=='Y') ? yStamp : (axis=='X') ? xStamp : wStamp;
  interrupts();
  if ((long)(micros() - t) > (long)STALE_US) return 0;   // stale
  if (w < 800)  w = 800;
  if (w > 2200) w = 2200;
  return w;
}
// ---------- end RC INPUT ----------

// ---------- Helpers ----------
static inline bool inDeadzone(unsigned long us){
  return (us > (unsigned long)DEADZONE_US_LOW && us < (unsigned long)DEADZONE_US_HIGH);
}
static inline float usToNorm(unsigned long us){
  if (us == 0) return 0.0f;
  if (inDeadzone(us)) return 0.0f;
  if (us < 1000UL) us = 1000UL;
  if (us > 2000UL) us = 2000UL;
  long span = (long)us - 1500L;   // -500..+500
  float v = (float)span / 500.0f; // -1..+1
  if (v > 1.0f) v = 1.0f;
  if (v < -1.0f) v = -1.0f;
  return v;
}
static inline void normalize4(float &a, float &b, float &c, float &d){
  float m = fabs(a);
  if (fabs(b) > m) m = fabs(b);
  if (fabs(c) > m) m = fabs(c);
  if (fabs(d) > m) m = fabs(d);
  if (m > 1.0f) { a/=m; b/=m; c/=m; d/=m; }
}
static inline int normToPWM(float v){
  if (v > 1.0f) v = 1.0f;
  if (v < -1.0f) v = -1.0f;
  return (int)roundf(v * 255.0f); // signed PWM
}
static inline int rampTo(int current, int target){
  if (current < target){ current += RAMP_STEP_PWM; if (current > target) current = target; }
  else if (current > target){ current -= RAMP_STEP_PWM; if (current < target) current = target; }
  return current;
}
static inline void driveMotor(Motor &m, int target){
  if (m.invert) target = -target;
  m.curr = rampTo(m.curr, target);
  if (m.curr > 0){
    digitalWrite(m.in1, HIGH); digitalWrite(m.in2, LOW);
    analogWrite(m.en, m.curr);
  } else if (m.curr < 0){
    digitalWrite(m.in1, LOW);  digitalWrite(m.in2, HIGH);
    analogWrite(m.en, -m.curr);
  } else {
    digitalWrite(m.in1, LOW);  digitalWrite(m.in2, LOW);
    analogWrite(m.en, 0);
  }
}
static inline void stopAll(){
  for (int i=0;i<4;i++) M[i].curr = 0;
  digitalWrite(LF_IN1,LOW); digitalWrite(LF_IN2,LOW); analogWrite(LF_EN,0);
  digitalWrite(RF_IN1,LOW); digitalWrite(RF_IN2,LOW); analogWrite(RF_EN,0);
  digitalWrite(LR_IN1,LOW); digitalWrite(LR_IN2,LOW); analogWrite(LR_EN,0);
  digitalWrite(RR_IN1,LOW); digitalWrite(RR_IN2,LOW); analogWrite(RR_EN,0);
}

// ---------- Setup / Loop ----------
void setup(){
  Serial.begin(115200);
  rcInputBegin();

  pinMode(LF_IN1,OUTPUT); pinMode(LF_IN2,OUTPUT); pinMode(LF_EN,OUTPUT);
  pinMode(LR_IN1,OUTPUT); pinMode(LR_IN2,OUTPUT); pinMode(LR_EN,OUTPUT);
  pinMode(RF_IN1,OUTPUT); pinMode(RF_IN2,OUTPUT); pinMode(RF_EN,OUTPUT);
  pinMode(RR_IN1,OUTPUT); pinMode(RR_IN2,OUTPUT); pinMode(RR_EN,OUTPUT);

  stopAll();
}

void loop(){
  // ---- Read RC (interrupt-backed) ----
  unsigned long usY = rcReadUs('Y'); // D0
  unsigned long usX = rcReadUs('X'); // D1
  unsigned long usW = rcReadUs('W'); // D7

  // Simple failsafe: if all stale, stop
  if (usX == 0 && usY == 0 && usW == 0){
    stopAll();
    delay(15);
    return;
  }

  // ---- Normalize + optional axis flips ----
  float X = usToNorm(usX);
  float Y = usToNorm(usY);
  float W = usToNorm(usW);
  if (INVERT_X) X = -X;
  if (INVERT_Y) Y = -Y;
  if (INVERT_W) W = -W;

  // ---- Mecanum mix (X,Y,W) ----
  float lf =  Y + X + W;
  float rf =  Y - X - W;
  float lr =  Y - X + W;
  float rr =  Y + X - W;

  normalize4(lf, rf, lr, rr);
  lf *= MAX_SCALE_MIX; rf *= MAX_SCALE_MIX; lr *= MAX_SCALE_MIX; rr *= MAX_SCALE_MIX;

  // ---- Drive with soft-start ----
  driveMotor(M[0], normToPWM(lf)); // LF
  driveMotor(M[1], normToPWM(rf)); // RF
  driveMotor(M[2], normToPWM(lr)); // LR
  driveMotor(M[3], normToPWM(rr)); // RR

  // ---- Debug ----
  Serial.print("usX:"); Serial.print(usX);
  Serial.print(" usY:"); Serial.print(usY);
  Serial.print(" usW:"); Serial.print(usW);
  Serial.print(" | X:"); Serial.print(X,2);
  Serial.print(" Y:"); Serial.print(Y,2);
  Serial.print(" W:"); Serial.print(W,2);
  Serial.print(" | LF:"); Serial.print(lf,2);
  Serial.print(" RF:"); Serial.print(rf,2);
  Serial.print(" LR:"); Serial.print(lr,2);
  Serial.print(" RR:"); Serial.println(rr,2);

  delay(15); // ~60–70 Hz loop
}
