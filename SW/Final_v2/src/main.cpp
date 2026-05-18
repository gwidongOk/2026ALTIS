#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <esp_timer.h>
#include <ESP32Servo.h>

#include "Config.h"
#include "LSM6DSO32.h"
#include "BMP388.h"
#include "NAV.h"
#include "MX25Logger.h"
#include "sensor_data.h"
#include "BLE.h"

// ============================================================
// Global objects
// ============================================================
static SPIClass sensorSPI(HSPI);
static SPIClass flashSPI(FSPI);

static LSM6DSO32 imu(IMU_CS_PIN, &sensorSPI);
static BMP388    bmp(BMP_CS_PIN, &sensorSPI);
static NAV       nav;
static MX25Logger logger;
static Servo     servoDrogue;

static TaskHandle_t TaskHandle_IMU   = NULL;
static TaskHandle_t TaskHandle_BMP   = NULL;
static TaskHandle_t FlightTaskHandle = NULL;
static TaskHandle_t FlushTaskHandle  = NULL;

static SemaphoreHandle_t spiMutex   = NULL;
static SemaphoreHandle_t navMutex   = NULL;
static SemaphoreHandle_t flashMutex = NULL;

static volatile bool flightActive = false;
static FlightPhase flightPhase = PRE_FLIGHT;

// Forward Declarations
void IMU_Task(void *pvParameters);
void BMP_Task(void *pvParameters);
void Flight_Task(void *pvParameters);
void FlushTask(void *pvParameters);
void beep(int ms, int count = 1);
void clearSensors();
void attachSensorInterrupts();
void detachSensorInterrupts();

// ============================================================
// setup() : Initialization
// ============================================================
void setup() {
  Serial.begin(SERIAL_BAUD);

  // 1. Pins & Feedback
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(PYRO_1_PIN, OUTPUT); pinMode(PYRO_2_PIN, OUTPUT);
  pinMode(SERVO_2_PIN, OUTPUT);
  pinMode(SERVO_3_PIN, OUTPUT); pinMode(SERVO_4_PIN, OUTPUT);

  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(PYRO_1_PIN, LOW); digitalWrite(PYRO_2_PIN, LOW);

  // SERVO_1 = drogue release (PWM)
  servoDrogue.attach(SERVO_1_PIN);
  servoDrogue.write(SERVO_DROGUE_IDLE_DEG);

  // FC-51 IR sensor for stage separation detection
  //   PULLDOWN: if sensor unpowered/disconnected, pin reads LOW = "not separated"
  //             → fails safe (no false ignition)
  pinMode(STAGE_IR_PIN, INPUT_PULLDOWN);

  // 2. Synchronizations
  spiMutex   = xSemaphoreCreateMutex();
  navMutex   = xSemaphoreCreateMutex();
  flashMutex = xSemaphoreCreateMutex();
  if (!spiMutex || !navMutex || !flashMutex) {
    beep(100, 3);
    while (1);
  }

  // 3. Communications
  initBLE(BLE_DEVICE_NAME);
  sensorSPI.begin(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN);

  // 4. Sensors
  bool ok = true;
  if (!imu.begin()) { sendResponse("IMU FAIL\n"); ok = false; }
  if (!bmp.begin()) { sendResponse("BMP FAIL\n"); ok = false; }

  if (!ok) { beep(100, 3); while(1); }
  sendResponse("IMU+BARO OK\n");

  // 5. Storage
  logger.begin(&flashSPI, FLASH_SCK_PIN, FLASH_MISO_PIN, FLASH_MOSI_PIN, FLASH_CS_PIN, flashMutex);

  // 6. Interrupts (Always active to keep NAV updated)
  pinMode(IMU_INT1_PIN, INPUT_PULLDOWN);
  pinMode(BMP_INT_PIN,  INPUT);
  attachSensorInterrupts();
  imu.enableAccelDataReadyInterrupt(1);

  // 7. Tasks
  xTaskCreatePinnedToCore(IMU_Task,    "IMU_T",   STACK_SIZE_SENSOR, NULL, TASK_C1_PRIO_IMU,   &TaskHandle_IMU,   1);
  xTaskCreatePinnedToCore(BMP_Task,    "BMP_T",   STACK_SIZE_SENSOR, NULL, TASK_C1_PRIO_BMP,   &TaskHandle_BMP,   1);
  xTaskCreatePinnedToCore(Flight_Task, "Flt_T",   4096,              NULL, 3,                   &FlightTaskHandle, 0);
  xTaskCreatePinnedToCore(FlushTask,   "Flush_T", STACK_SIZE_FLUSH,  NULL, TASK_C0_PRIO_FLUSH,  &FlushTaskHandle,  0);

  beep(200);
  sendResponse(">>> 1-AXIS READY\n");
}

// ============================================================
// loop() : Command Dispatcher
// ============================================================
void loop() {
  String cmd = getIncomingRaw();
  if (cmd.length() == 0) { vTaskDelay(pdMS_TO_TICKS(50)); return; }
  cmd.toUpperCase();

  // --- Common Commands ---
  if (cmd == "REBOOT") {
    sendResponse("REBOOTING...\n");
    if (logger.isEnabled()) {
      logger.setEnabled(false);
      vTaskDelay(pdMS_TO_TICKS(100));
      logger.forceFlushBuffer();
    }
    vTaskDelay(pdMS_TO_TICKS(200));
    ESP.restart();
  }

  else if (cmd == "PARSE") {
    // PARSE works in any state (pre-flight, in-flight, after STOP).
    bool wasLogging = logger.isEnabled();
    if (wasLogging) {
      logger.setEnabled(false);
      vTaskDelay(pdMS_TO_TICKS(100));
    }
    sendResponse("DUMP START\n");
    logger.forceFlushBuffer();
    logger.dumpRawBinary(Serial);
    sendResponse("DUMP DONE\n");
    if (wasLogging) { flightActive = false; }
  }

  // --- Pre-Flight Commands ---
  else if (!flightActive) {
    if (cmd == "CALIBRATE") {
      bool imuOk = false, bmpOk = false;
      while (!(imuOk && bmpOk)) {
        sendResponse("CALIBRATING IMU+BMP...\n");
        // Hold spiMutex for the full calibration: imu/bmp.calibrate() read SPI
        // directly without locking, and would race with IMU_Task/BMP_Task.
        xSemaphoreTake(spiMutex, portMAX_DELAY);
        imuOk = imu.calibrate(CALIB_SAMPLES);
        bmpOk = bmp.calibrate(CALIB_SAMPLES);
        xSemaphoreGive(spiMutex);
        clearSensors();

        if (imuOk && bmpOk) {
          sendResponse("CALIBRATION DONE.\n");
          beep(200);
        } else {
          if (!imuOk) sendResponse("IMU NOISY - RETRYING...\n");
          if (!bmpOk) sendResponse("BARO UNSTABLE - RETRYING...\n");
          beep(100, 3);

          // Allow REBOOT to abort the retry loop
          String c2 = getIncomingRaw();
          c2.toUpperCase();
          if (c2 == "REBOOT") {
            sendResponse("CALIBRATION ABORTED - REBOOTING...\n");
            vTaskDelay(pdMS_TO_TICKS(200));
            ESP.restart();
          }
          vTaskDelay(pdMS_TO_TICKS(1000));
        }
      }
    }
    else if (cmd == "ERASE") {
      sendResponse("ERASING FLASH...\n");
      logger.eraseAll();
      sendResponse("DONE.\n");
      beep(500);
    }

    // ── Actuator tests (pre-flight only) ──────────────────────────────
    else if (cmd == "TEST SERVO") {
      sendResponse("SERVO: IDLE -> DEPLOY -> IDLE\n");
      servoDrogue.write(SERVO_DROGUE_DEPLOY_DEG);
      vTaskDelay(pdMS_TO_TICKS(1000));
      servoDrogue.write(SERVO_DROGUE_IDLE_DEG);
      sendResponse("SERVO OK\n");
      beep(100, 2);
    }
    else if (cmd == "TEST PYRO1") {
      // !! e-match must be DISCONNECTED before running this !!
      sendResponse("PYRO1: 100ms pulse (e-match disconnected?)\n");
      beep(50, 3);                             // triple-beep warning
      vTaskDelay(pdMS_TO_TICKS(1000));         // 1s delay for operator to stand clear
      digitalWrite(PYRO_1_PIN, HIGH);
      vTaskDelay(pdMS_TO_TICKS(100));
      digitalWrite(PYRO_1_PIN, LOW);
      sendResponse("PYRO1 DONE\n");
      beep(200);
    }
    else if (cmd == "TEST PYRO2") {
      // !! e-match must be DISCONNECTED before running this !!
      sendResponse("PYRO2: 100ms pulse (e-match disconnected?)\n");
      beep(50, 3);
      vTaskDelay(pdMS_TO_TICKS(1000));
      digitalWrite(PYRO_2_PIN, HIGH);
      vTaskDelay(pdMS_TO_TICKS(100));
      digitalWrite(PYRO_2_PIN, LOW);
      sendResponse("PYRO2 DONE\n");
      beep(200);
    }

    else if (cmd == "START") {
      sendResponse("STARTING...\n");
      // kfBegin polls _state_imu for ~250ms; do NOT hold navMutex during
      // the loop or IMU_Task gets starved and only 1 sample is collected.
      nav.kfBegin();
      sendResponse("KF READY\n");

      clearSensors();
      logger.setEnabled(true);
      flightActive = true;
      clearSensors();
      beep(300);
      sendResponse("FLIGHT ACTIVE\n");
      xTaskNotifyGive(FlightTaskHandle);
    }
  }

  // --- Flight/Active Commands ---
  else {
    if (cmd == "STOP") {
      if (logger.isEnabled()) {
        logger.setEnabled(false);
        vTaskDelay(pdMS_TO_TICKS(100));
        logger.forceFlushBuffer();

        flightActive = false;
        beep(200); vTaskDelay(pdMS_TO_TICKS(50)); beep(200);
        sendResponse("STOPPED.\n");
      }
    }
    // PARSE handled in common commands above (works regardless of flightActive)
  }

  vTaskDelay(pdMS_TO_TICKS(50));
}

// ============================================================
// Flight Control Task (Core 0, 10Hz)
// ============================================================
void Flight_Task(void *pvParameters) {
  static uint32_t pyro1_start_ms = 0;
  static uint32_t pyro2_start_ms = 0;
  static bool     pyro1_active = false;
  static bool     pyro2_active = false;
  static bool     stage2_attempted = false;
  static bool     drogue_deployed  = false;
  static bool     main_deployed    = false;

  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // Reset per-flight state
    flightPhase      = PRE_FLIGHT;
    pyro1_active     = false;
    pyro2_active     = false;
    stage2_attempted = false;
    drogue_deployed  = false;
    main_deployed    = false;
    uint8_t launch_count = 0;
    uint8_t descent_count = 0;
    uint8_t landed_count  = 0;
    uint8_t sep_count     = 0;

    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (flightActive) {
      vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100)); // 10Hz

      uint32_t now_ms = millis();

      // Pyro auto-off timers
      if (pyro1_active && (now_ms - pyro1_start_ms > STAGE2_PULSE_MS)) {
        digitalWrite(PYRO_1_PIN, LOW);
        pyro1_active = false;
      }
      if (pyro2_active && (now_ms - pyro2_start_ms > MAIN_PULSE_MS)) {
        digitalWrite(PYRO_2_PIN, LOW);
        pyro2_active = false;
      }

      // Consistent state snapshot
      State_nominal s;
      State_imu si;
      xSemaphoreTake(navMutex, portMAX_DELAY);
      s  = nav.getNominal();
      si = nav.getStateImu();
      xSemaphoreGive(navMutex);

      float alt    = -s.p[2];
      float vel_up = -s.v[2];

      // Specific force magnitude: ≈0 in free-fall, ≈g at rest, ≫g under thrust
      float fmag = sqrtf(si.ax * si.ax + si.ay * si.ay + si.az * si.az);

      switch (flightPhase) {
        case PRE_FLIGHT:
          // Debounced: 3 consecutive samples @ 10Hz = 300 ms above threshold
          if (alt > 5.0f || vel_up > 5.0f) {
            if (++launch_count >= 3) {
              flightPhase = POWERED_FLIGHT;
              logger.logEvent(flightPhase, 1);
              sendResponse("LAUNCH\n");
              launch_count = 0;
            }
          } else {
            launch_count = 0;
          }
          break;

        case POWERED_FLIGHT:
          if (fmag < 2.0f && vel_up > 10.0f) {
            flightPhase = COASTING;
            logger.logEvent(flightPhase, 2);
            sendResponse("BO\n");
          }
          break;

        case COASTING: {
          if (!stage2_attempted) {
            sep_count = (digitalRead(STAGE_IR_PIN) == HIGH) ? sep_count + 1 : 0;
            if (sep_count >= 3) {
              // Tilt check via quaternion (no gimbal lock):
              // cos(tilt) = -R[2][0] = 2*(qw*qy - qx*qz)
              // 1 = vertical, 0 = horizontal, -1 = inverted
              float cos_tilt = 2.0f * (s.q[0]*s.q[2] - s.q[1]*s.q[3]);
              bool  tilt_ok  = (cos_tilt > cosf(TILT_LIMIT_DEG * (float)M_PI / 180.0f));
              if (tilt_ok) {
                stage2_attempted = true;
                digitalWrite(PYRO_1_PIN, HIGH);
                pyro1_start_ms = now_ms;
                pyro1_active   = true;
                sendResponse("STAGE2 IGN\n");
              } else {
                // Separation confirmed but tilt exceeds limit — log each cycle
                logger.logEvent(flightPhase, 5);  // NotStageCondition
                sendResponse("NSC TILT\n");
              }
            }
          }

          // Apogee detection (debounced: 3 samples = 300 ms)
          if (vel_up < -0.5f && alt > 15.0f) {
            if (++descent_count >= 3) {
              if (!stage2_attempted) {
                logger.logEvent(flightPhase, 5);  // NSC: no separation before apogee
                sendResponse("NSC NOSEP\n");
              }
              flightPhase = DESCENT;
              logger.logEvent(flightPhase, 3);
              sendResponse("APG\n");
              drogue_deployed = true;
              servoDrogue.write(SERVO_DROGUE_DEPLOY_DEG);
              sendResponse("DROGUE\n");
              descent_count = 0;
            }
          } else {
            descent_count = 0;
          }
          break;
        }

        case DESCENT:
          // Main parachute at altitude threshold
          if (!main_deployed && alt < MAIN_DEPLOY_ALT_M) {
            main_deployed  = true;
            digitalWrite(PYRO_2_PIN, HIGH);
            pyro2_start_ms = now_ms;
            pyro2_active   = true;
            sendResponse("MAIN\n");
          }

          // Landed detection (debounced: 10 samples @ 10Hz = 1s)
          if (alt < 10.0f && fabsf(vel_up) < 1.0f) {
            if (++landed_count >= 10) {
              flightPhase = LANDED;
              logger.logEvent(flightPhase, 4);
              sendResponse("LAND\n");
              logger.setEnabled(false);
              vTaskDelay(pdMS_TO_TICKS(100));
              logger.forceFlushBuffer();
              flightActive = false;
              beep(500, 3);
              sendResponse("STOPPED.\n");
              landed_count = 0;
            }
          } else {
            landed_count = 0;
          }
          break;

        case LANDED:
          break;
      }
    }
  }
}

// ============================================================
// Sensor & Utility Tasks (Core 1)
// ============================================================
void IMU_Task(void *pvParameters) {
  Raw_imu raw;
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    if (xSemaphoreTake(spiMutex, portMAX_DELAY) == pdTRUE) {
      raw.timestamp = (uint32_t)(esp_timer_get_time() & 0xFFFFFFFF);
      imu.readCalibratedIMU(raw.gx, raw.gy, raw.gz, raw.ax, raw.ay, raw.az);
      xSemaphoreGive(spiMutex);
    }

    if (flightActive) logger.logImu(raw);

    if (xSemaphoreTake(navMutex, portMAX_DELAY) == pdTRUE) {
      nav.updateIMU(raw);
      if (flightActive && nav.isKfReady()) {
        logger.logState(nav.getNominal());
      }
      xSemaphoreGive(navMutex);
    }
  }
}

void BMP_Task(void *pvParameters) {
  Raw_press p;
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    if (xSemaphoreTake(spiMutex, portMAX_DELAY) == pdTRUE) {
      p.timestamp = (uint32_t)(esp_timer_get_time() & 0xFFFFFFFF);
      bmp.readAltitude(p.alt);
      xSemaphoreGive(spiMutex);
    }

    if (flightActive) logger.logBaro(p);

    if (xSemaphoreTake(navMutex, portMAX_DELAY) == pdTRUE) {
      nav.updatePress(p);
      xSemaphoreGive(navMutex);
    }
  }
}

void FlushTask(void *pvParameters) {
  for (;;) { logger.serviceFlush(); }
}

// ============================================================
// Utilities
// ============================================================
void beep(int ms, int count) {
  for (int i = 0; i < count; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    vTaskDelay(pdMS_TO_TICKS(ms));
    digitalWrite(BUZZER_PIN, LOW);
    if (i < count - 1) vTaskDelay(pdMS_TO_TICKS(ms));
  }
}

void clearSensors() {
  int16_t d1, d2, d3, d4, d5, d6; float df;
  xSemaphoreTake(spiMutex, portMAX_DELAY);
  imu.readRawIMU(d1, d2, d3, d4, d5, d6);
  bmp.readData(df);
  xSemaphoreGive(spiMutex);
}

void IRAM_ATTR IMUInterruptHandler() {
  BaseType_t woken = pdFALSE;
  if (TaskHandle_IMU) vTaskNotifyGiveFromISR(TaskHandle_IMU, &woken);
  portYIELD_FROM_ISR(woken);
}

void IRAM_ATTR BMPInterruptHandler() {
  BaseType_t woken = pdFALSE;
  if (TaskHandle_BMP) vTaskNotifyGiveFromISR(TaskHandle_BMP, &woken);
  portYIELD_FROM_ISR(woken);
}

void attachSensorInterrupts() {
  attachInterrupt(digitalPinToInterrupt(IMU_INT1_PIN), IMUInterruptHandler, RISING);
  attachInterrupt(digitalPinToInterrupt(BMP_INT_PIN),  BMPInterruptHandler,  RISING);
}

void detachSensorInterrupts() {
  detachInterrupt(digitalPinToInterrupt(IMU_INT1_PIN));
  detachInterrupt(digitalPinToInterrupt(BMP_INT_PIN));
}
