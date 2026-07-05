// ============================================================
// GROUND TEST — Hall-sensor-triggered actuator sequence test
//
// Purpose: bench-test the stage2/drogue/main actuators and confirm the
// Hall sensor's actual polarity, without needing a real flight (no IMU/
// baro/NAV involved). Standalone sketch — does NOT touch Final_v3/src,
// build/flash this independently (Arduino IDE: install "ESP32Servo").
//
// Sequence:
//   Hall sensor separated (3 samples @ 10Hz = 300ms debounce, same as
//   flight code's sep_count>=3)
//     -> immediately: Stage-2 ignition (PYRO_1)
//     -> +9.5s        : simulated apogee -> Drogue release (servo)
//     -> +5.0s more   : Main deploy (PYRO_2)
//
// Watch the Serial monitor: the raw HALL_STATE line lets you wave a
// magnet at the sensor and see whether HIGH means "separated" for your
// actual hardware — if it reads backwards, flip SEP_ACTIVE_STATE below.
// ============================================================
#include <Arduino.h>
#include <ESP32Servo.h>

// ---- Pins (mirror Config.h in Final_v3/src) ----
#define STAGE_SEP_PIN   1    // Hall sensor
#define PYRO_1_PIN      18   // Stage-2 re-ignition
#define PYRO_2_PIN      8    // Main deploy
#define SERVO_1_PIN     41   // Drogue release
#define BUZZER_PIN      17

// ---- Timing / actuator constants (mirror Config.h) ----
#define STAGE2_PULSE_MS         1000
#define MAIN_PULSE_MS           1000
#define SERVO_DROGUE_IDLE_DEG   90
#define SERVO_DROGUE_DEPLOY_DEG 0

// ---- Test-specific parameters ----
#define SEP_DEBOUNCE_COUNT   3      // matches flight code's sep_count>=3
#define APOGEE_DELAY_MS      9500   // ground-test stand-in for real apogee detection
#define MAIN_DELAY_MS        5000   // after drogue

// Open-drain/active-low Hall sensor with an external 4.7k pull-up already
// on the board (do not add an internal pull, it fights the external one).
// KNOWN GAP: with the magnet mounted so separation moves it away from the
// sensor, "separated" and "sensor disconnected" both read HIGH and can't
// be told apart here. Real fix is remounting the magnet the other way
// (LOW = separated) — see main.cpp's setup() comment in Final_v3/src.
#define SEP_ACTIVE_STATE  HIGH

Servo servoDrogue;

enum TestPhase : uint8_t { WAIT_SEP, WAIT_APOGEE, WAIT_MAIN, DONE };
static TestPhase phase = WAIT_SEP;

static uint8_t  sepCount        = 0;
static uint32_t sepDetectedMs   = 0;
static uint32_t apogeeMs        = 0;

static bool     pyro1Active = false;
static uint32_t pyro1StartMs = 0;
static bool     pyro2Active = false;
static uint32_t pyro2StartMs = 0;

void beep(int ms, int count = 1) {
  for (int i = 0; i < count; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(ms);
    digitalWrite(BUZZER_PIN, LOW);
    if (i < count - 1) delay(ms);
  }
}

void setup() {
  Serial.begin(921600);

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  pinMode(PYRO_1_PIN, OUTPUT);
  pinMode(PYRO_2_PIN, OUTPUT);
  digitalWrite(PYRO_1_PIN, LOW);
  digitalWrite(PYRO_2_PIN, LOW);

  // External pull-up already on the board — no internal pull here.
  pinMode(STAGE_SEP_PIN, INPUT);

  servoDrogue.attach(SERVO_1_PIN);
  servoDrogue.write(SERVO_DROGUE_IDLE_DEG);

  beep(200);
  Serial.println(">>> GROUND TEST READY - waiting for Hall sensor separation");
}

void loop() {
  uint32_t now = millis();

  // Pyro auto-off timers (same pattern as Flight_Task)
  if (pyro1Active && now - pyro1StartMs > STAGE2_PULSE_MS) {
    digitalWrite(PYRO_1_PIN, LOW);
    pyro1Active = false;
  }
  if (pyro2Active && now - pyro2StartMs > MAIN_PULSE_MS) {
    digitalWrite(PYRO_2_PIN, LOW);
    pyro2Active = false;
  }

  bool sepState = (digitalRead(STAGE_SEP_PIN) == SEP_ACTIVE_STATE);
  Serial.printf("[HALL_STATE] raw=%d active=%d phase=%d count=%u\n",
                digitalRead(STAGE_SEP_PIN), sepState, (int)phase, sepCount);

  switch (phase) {
    case WAIT_SEP:
      sepCount = sepState ? sepCount + 1 : 0;
      if (sepCount >= SEP_DEBOUNCE_COUNT) {
        Serial.println(">>> SEPARATION DETECTED -> STAGE2 IGNITION");
        digitalWrite(PYRO_1_PIN, HIGH);
        pyro1StartMs = now;
        pyro1Active  = true;
        sepDetectedMs = now;
        beep(100, 2);
        phase = WAIT_APOGEE;
      }
      break;

    case WAIT_APOGEE:
      if (now - sepDetectedMs >= APOGEE_DELAY_MS) {
        Serial.println(">>> APOGEE (simulated) -> DROGUE");
        servoDrogue.write(SERVO_DROGUE_DEPLOY_DEG);
        apogeeMs = now;
        beep(200);
        phase = WAIT_MAIN;
      }
      break;

    case WAIT_MAIN:
      if (now - apogeeMs >= MAIN_DELAY_MS) {
        Serial.println(">>> MAIN DEPLOY");
        digitalWrite(PYRO_2_PIN, HIGH);
        pyro2StartMs = now;
        pyro2Active  = true;
        beep(500, 3);
        phase = DONE;
      }
      break;

    case DONE:
      break;
  }

  delay(100);
}
