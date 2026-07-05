// ============================================================
// GROUND TEST — Hall sensor -> stage2 -> drogue -> main, on a timer.
// Standalone sketch (Arduino IDE, install "ESP32Servo"). Does not touch
// Final_v3/src.
//
// loop() just polls the Hall sensor at 10Hz and debounces it. Once
// separation is confirmed, the rest of the sequence runs once, straight
// through, using delay() between each step (no going back through loop()):
//   Hall sensor separated (3 samples @ 10Hz, debounced)
//     -> immediately : Stage-2 ignition (PYRO_1)
//     -> +9.5s        : simulated apogee -> Drogue (servo)
//     -> +5.0s more   : Main deploy (PYRO_2)
//
// Watch [HALL_STATE] on Serial while moving the actual mechanism to
// confirm SEP_ACTIVE_STATE below is right for this sensor/wiring.
// ============================================================
#include <Arduino.h>
#include <ESP32Servo.h>

#define STAGE_SEP_PIN  1
#define PYRO_1_PIN     18
#define PYRO_2_PIN     8
#define SERVO_1_PIN    41
#define BUZZER_PIN     17

#define STAGE2_PULSE_MS   1000
#define MAIN_PULSE_MS     1000
#define SERVO_IDLE_DEG    90
#define SERVO_DEPLOY_DEG  0

#define APOGEE_DELAY_MS  9500   // after separation is detected
#define MAIN_DELAY_MS    5000   // after drogue


Servo   servoDrogue;
uint8_t sepCount = 0;

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
  pinMode(PYRO_1_PIN, OUTPUT);
  pinMode(PYRO_2_PIN, OUTPUT);
  digitalWrite(PYRO_1_PIN, LOW);
  digitalWrite(PYRO_2_PIN, LOW);
  pinMode(STAGE_SEP_PIN, INPUT);

  servoDrogue.attach(SERVO_1_PIN);
  servoDrogue.write(SERVO_IDLE_DEG);

  beep(200);
  while(HIGH == digitalRead(STAGE_SEP_PIN)){
    Serial.println("hall sensor say I am seperated");
    delay(100);
  }
  Serial.println(">>> GROUND TEST READY - waiting for Hall sensor separation");
}

void loop() {
  if(1 == digitalRead(STAGE_SEP_PIN)){
    sepCount += 1;
  }
  else{
    sepCount = 0;
  }
  Serial.println(sepCount);

  if (sepCount >= 3) {
    Serial.println(">>> SEPARATION DETECTED -> STAGE2 IGNITION");
    digitalWrite(PYRO_1_PIN, HIGH); //2단부 점화
    beep(100, 2);
    delay(STAGE2_PULSE_MS);
    digitalWrite(PYRO_1_PIN, LOW);

    delay(APOGEE_DELAY_MS - STAGE2_PULSE_MS); //9.5초 뒤
    Serial.println(">>> APOGEE (simulated) -> DROGUE");
    servoDrogue.write(SERVO_DEPLOY_DEG);//낙하산 사출장치
    beep(200);

    delay(MAIN_DELAY_MS);
    Serial.println(">>> MAIN DEPLOY");
    digitalWrite(PYRO_2_PIN, HIGH);//형상변화
    beep(500, 3);
    delay(MAIN_PULSE_MS);
    digitalWrite(PYRO_2_PIN, LOW);

    Serial.println(">>> TEST SEQUENCE DONE");
    while (1){
      delay(100);
    }
  }

  delay(100);
}
