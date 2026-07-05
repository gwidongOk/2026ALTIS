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
// Serializes the four launch/disarm state-transition functions
// (beginLaunchVerification/confirmLaunch/rejectFalseLaunch/disarmSystem) so
// a DISARM from loop() (core 0) can never interleave with a launch decision
// made by IMU_Task (core 1) on systemMode/flightActive/nav/logger/preTrigger.
static SemaphoreHandle_t stateMutex = NULL;

static volatile bool flightActive = false;
static bool calibrationDone = false;
static bool flashErasedThisBoot = false;
static FlightPhase flightPhase = PRE_FLIGHT;

enum class SystemMode : uint8_t {
  IDLE = 0,
  READY_CONVERGING,
  ARMED,
  LAUNCH_VERIFY,
  LAUNCH_COMMIT,
  FLIGHT,
  LANDED
};

static volatile SystemMode systemMode = SystemMode::IDLE;
static portMUX_TYPE modeMux = portMUX_INITIALIZER_UNLOCKED;
static PreTriggerBuffer preTrigger;

struct LaunchDetectorState {
  bool candidate = false;
  uint32_t firstTimestampUs = 0;
  uint32_t lastTimestampUs = 0;
  uint32_t candidateStartMs = 0;
  uint32_t belowResetStartMs = 0;
  float deltaV = 0.0f;
};

static LaunchDetectorState launchDetector;
static volatile uint32_t launchStartMs = 0;

// Forward Declarations
void IMU_Task(void *pvParameters);
void BMP_Task(void *pvParameters);
void Flight_Task(void *pvParameters);
void FlushTask(void *pvParameters);
void beep(int ms, int count = 1);
void clearSensors();
void attachSensorInterrupts();
void detachSensorInterrupts();
SystemMode getSystemMode();
void setSystemMode(SystemMode mode);
void resetLaunchDetector();
bool updateLaunchDetector(const Raw_imu &raw);
void beginLaunchVerification(const Raw_imu &raw);
void confirmLaunch();
void rejectFalseLaunch(uint32_t timestamp);
void disarmSystem();

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

  // Hall sensor for stage separation detection (open-drain, active-low,
  // external 4.7k pull-up already on the board — do NOT add an internal
  // pull here, it would fight the external one).
  //   KNOWN GAP: with the magnet mounted so separation moves it AWAY from
  //   the sensor (current build), "separated" and "sensor
  //   disconnected/unpowered" both read HIGH and are indistinguishable in
  //   software. Fix is mechanical: remount the magnet so separation brings
  //   it INTO range instead (LOW = separated), so any failure defaults to
  //   the safe "not separated" reading. Until that remount happens, this
  //   pin cannot fail safe against a disconnected sensor.
  pinMode(STAGE_SEP_PIN, INPUT);

  // 2. Synchronizations
  spiMutex   = xSemaphoreCreateMutex();
  navMutex   = xSemaphoreCreateMutex();
  flashMutex = xSemaphoreCreateMutex();
  stateMutex = xSemaphoreCreateMutex();
  if (!spiMutex || !navMutex || !flashMutex || !stateMutex) {
    beep(100, 3);
    while (1);
  }
  if (!preTrigger.begin(PRETRIGGER_RECORD_CAPACITY)) {
    sendResponse("PRETRIGGER PSRAM FAIL\n");
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
  if (!logger.begin(&flashSPI, FLASH_SCK_PIN, FLASH_MISO_PIN,
                    FLASH_MOSI_PIN, FLASH_CS_PIN, flashMutex)) {
    sendResponse("LOGGER INIT FAIL\n");
    beep(100, 3);
    while (1);
  }

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
  SystemMode mode = getSystemMode();

  // --- Common Commands ---
  if (cmd == "REBOOT") {
    digitalWrite(PYRO_1_PIN, LOW);
    digitalWrite(PYRO_2_PIN, LOW);
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
    if (mode != SystemMode::IDLE && mode != SystemMode::LANDED) {
      sendResponse("PARSE BLOCKED - DISARM OR LAND FIRST\n");
    } else {
      sendResponse("DUMP START\n");
      logger.forceFlushBuffer();
      logger.dumpRawBinary(Serial);
      sendResponse("DUMP DONE\n");
    }
  }

  else if (cmd == "DISARM") {
    disarmSystem();
  }

  else if (cmd == "STOP") {
    digitalWrite(PYRO_1_PIN, LOW);
    digitalWrite(PYRO_2_PIN, LOW);
    if (logger.isEnabled()) {
      logger.setEnabled(false);
      vTaskDelay(pdMS_TO_TICKS(100));
      logger.forceFlushBuffer();
    }
    flightActive = false;
    setSystemMode(SystemMode::LANDED);
    beep(200); vTaskDelay(pdMS_TO_TICKS(50)); beep(200);
    sendResponse("STOPPED.\n");
  }

  // --- Pre-Flight Commands ---
  else if (!flightActive) {
    bool armSequenceActive =
        mode == SystemMode::READY_CONVERGING ||
        mode == SystemMode::ARMED ||
        mode == SystemMode::LAUNCH_VERIFY ||
        mode == SystemMode::LAUNCH_COMMIT;

    if (armSequenceActive) {
      if (cmd == "READY") {
        char status[128];
        snprintf(status, sizeof(status), "READY MODE %u\n", (unsigned)mode);
        sendResponse(status);
      } else {
        sendResponse("READY ACTIVE - ONLY DISARM OR REBOOT\n");
      }
    }
    else if (cmd == "CALIBRATE") {
      bool imuOk = false;
      calibrationDone = false;
      while (!imuOk) {
        sendResponse("IMU WARMUP (5s)...\n");
        vTaskDelay(pdMS_TO_TICKS(IMU_CALIB_WARMUP_MS));
        sendResponse("CALIBRATING IMU (10s)...\n");
        // The vehicle is in the known 90-degree vertical pose. This is the
        // only stage that estimates accelerometer bias.
        xSemaphoreTake(spiMutex, portMAX_DELAY);
        imuOk = imu.calibrate(IMU_CALIB_SAMPLES);
        xSemaphoreGive(spiMutex);
        clearSensors();

        // Report IMU stats (measured vs threshold) over BLE so the operator
        // can see why a calibration attempt fails.
        const LSM6DSO32::CalibStats& cs = imu.getLastCalibStats();
        char buf[200];
        snprintf(buf, sizeof(buf),
                 "GYRO STD: %.2f %.2f %.2f (lim %.1f)\n",
                 cs.std_g[0], cs.std_g[1], cs.std_g[2],
                 LSM6DSO32::MAX_GYRO_STD_LSB);
        sendResponse(buf);
        snprintf(buf, sizeof(buf),
                 "ACCEL STD: %.2f %.2f %.2f (lim %.1f)\n",
                 cs.std_a[0], cs.std_a[1], cs.std_a[2],
                 LSM6DSO32::MAX_ACCEL_STD_LSB);
        sendResponse(buf);

        if (imuOk) {
          calibrationDone = true;
          sendResponse("CALIBRATION DONE.\n");
          beep(200);
        } else {
          sendResponse("IMU NOISY - RETRYING...\n");
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
      flashErasedThisBoot = true;
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
      sendResponse("PYRO1: 1000ms pulse (e-match disconnected?)\n");
      beep(50, 3);                             // triple-beep warning
      vTaskDelay(pdMS_TO_TICKS(1000));         // 1s delay for operator to stand clear
      digitalWrite(PYRO_1_PIN, HIGH);
      vTaskDelay(pdMS_TO_TICKS(STAGE2_PULSE_MS));
      digitalWrite(PYRO_1_PIN, LOW);
      sendResponse("PYRO1 DONE\n");
      beep(200);
    }
    else if (cmd == "TEST PYRO2") {
      // !! e-match must be DISCONNECTED before running this !!
      sendResponse("PYRO2: 1000ms pulse (e-match disconnected?)\n");
      beep(50, 3);
      vTaskDelay(pdMS_TO_TICKS(1000));
      digitalWrite(PYRO_2_PIN, HIGH);
      vTaskDelay(pdMS_TO_TICKS(MAIN_PULSE_MS));
      digitalWrite(PYRO_2_PIN, LOW);
      sendResponse("PYRO2 DONE\n");
      beep(200);
    }

    else if (cmd == "READY") {
      if (!calibrationDone) {
        sendResponse("CALIBRATE REQUIRED\n");
        vTaskDelay(pdMS_TO_TICKS(50));
        return;
      }
      if (!flashErasedThisBoot) {
        sendResponse("ERASE REQUIRED\n");
        vTaskDelay(pdMS_TO_TICKS(50));
        return;
      }
      // Bench-confirmed sensor behavior: magnet near (joined) = HIGH,
      // magnet far (separated) = LOW; disconnected also floats HIGH via
      // the external pull-up. So LOW here means already separated (or a
      // short) — neither should be armed. (Only checked at this instant;
      // does not cover a failure that happens later during the
      // READY/ARMED hold or in flight.)
      if (digitalRead(STAGE_SEP_PIN) == LOW) {
        sendResponse("STAGE SEP SENSOR NOT HIGH - CHECK WIRING/MAGNET\n");
        vTaskDelay(pdMS_TO_TICKS(50));
        return;
      }
      sendResponse("READY ALIGNING...\n");
      // kfBegin polls _state_imu for about 1s; do NOT hold navMutex during
      // the loop or IMU_Task gets starved and alignment cannot collect samples.
      while (!nav.kfBegin()) {
        sendResponse("KF ALIGN RETRY\n");
        beep(100, 3);

        String c2 = getIncomingRaw();
        c2.toUpperCase();
        if (c2 == "REBOOT") {
          sendResponse("READY ABORTED - REBOOTING...\n");
          vTaskDelay(pdMS_TO_TICKS(200));
          ESP.restart();
        }
        vTaskDelay(pdMS_TO_TICKS(500));
      }

      // The initial attitude is now fixed at the actual launch-rail angle.
      // Re-zero barometric altitude in this final pre-launch pose.
      bool baroOk = false;
      Raw_press zeroPress = {};
      while (!baroOk) {
        sendResponse("READY BARO ZEROING...\n");
        xSemaphoreTake(spiMutex, portMAX_DELAY);
        baroOk = bmp.calibrate(BARO_ZERO_SAMPLES);
        if (baroOk) {
          zeroPress.timestamp = (uint32_t)(esp_timer_get_time() & 0xFFFFFFFF);
          baroOk = bmp.readAltitude(zeroPress.alt);
          if (baroOk) bmp.startReferenceTracking(READY_BARO_REF_TAU_S);
        }
        xSemaphoreGive(spiMutex);

        if (!baroOk) {
          sendResponse("BARO UNSTABLE - RETRYING...\n");
          beep(100, 3);

          String c2 = getIncomingRaw();
          c2.toUpperCase();
          if (c2 == "REBOOT") {
            sendResponse("READY ABORTED - REBOOTING...\n");
            vTaskDelay(pdMS_TO_TICKS(200));
            ESP.restart();
          }
          vTaskDelay(pdMS_TO_TICKS(1000));
        }
      }

      xSemaphoreTake(navMutex, portMAX_DELAY);
      nav.updatePress(zeroPress);
      nav.beginZupt();
      xSemaphoreGive(navMutex);
      preTrigger.clear();
      resetLaunchDetector();
      setSystemMode(SystemMode::READY_CONVERGING);
      beep(200, 2);
      sendResponse("READY CONVERGING - WAIT FOR ARMED\n");
    }

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
  static uint32_t powered_start_ms = 0;

  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // Reset per-flight state
    flightPhase      = POWERED_FLIGHT;
    pyro1_active     = false;
    pyro2_active     = false;
    stage2_attempted = false;
    drogue_deployed  = false;
    main_deployed    = false;
    powered_start_ms = launchStartMs;
    uint8_t descent_count = 0;
    uint8_t landed_count  = 0;
    uint8_t sep_count     = 0;

    logger.logEvent(flightPhase, 1);
    sendResponse("LAUNCH CONFIRMED\n");

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
          // Launch detection is completed before this task is notified.
          break;

        case POWERED_FLIGHT: {
          bool burnout_detected = (fmag < 2.0f && vel_up > 10.0f);
          bool burnout_timeout  = (powered_start_ms != 0 &&
                                   now_ms - powered_start_ms >= BURNOUT_TIMEOUT_MS);
          if (burnout_detected || burnout_timeout) {
            flightPhase = COASTING;
            logger.logEvent(flightPhase, 2);
            sendResponse(burnout_timeout && !burnout_detected ? "BO TIMEOUT\n" : "BO\n");
          }
          break;
        }

        case COASTING: {
          if (!stage2_attempted) {
            sep_count = (digitalRead(STAGE_SEP_PIN) == LOW) ? sep_count + 1 : 0; //단분리 인지(LOW) 0.3초후
            if (sep_count >= 3) {
              // Separation is confirmed. Make exactly one ignition decision;
              // a failed tilt check permanently inhibits stage-2 ignition.
              stage2_attempted = true;

              // Tilt check via quaternion (no gimbal lock):
              // cos(tilt) = -R[2][0] = 2*(qw*qy - qx*qz)
              // 1 = vertical, 0 = horizontal, -1 = inverted
              float cos_tilt = 2.0f * (s.q[0]*s.q[2] - s.q[1]*s.q[3]);
              bool  tilt_ok  = (cos_tilt > cosf(TILT_LIMIT_DEG * (float)M_PI / 180.0f));
              if (tilt_ok) {
                digitalWrite(PYRO_1_PIN, HIGH);
                pyro1_start_ms = now_ms;
                pyro1_active   = true;
                logger.logEvent(flightPhase, 6);  // Stage2Ignition
                sendResponse("STAGE2 IGN\n");
              } else {
                // Separation confirmed but tilt exceeds limit — permanent abort
                logger.logEvent(flightPhase, 5);  // NotStageCondition
                sendResponse("NSC TILT - STAGE2 ABORT\n");
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
              setSystemMode(SystemMode::LANDED);
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

    SystemMode mode = getSystemMode();
    bool preflightBuffering =
        mode == SystemMode::READY_CONVERGING ||
        mode == SystemMode::ARMED ||
        mode == SystemMode::LAUNCH_VERIFY;
    if (preflightBuffering) preTrigger.pushImu(raw);

    bool launchCandidate = false;
    if (mode == SystemMode::ARMED) {
      launchCandidate = updateLaunchDetector(raw);
    }

    bool becameArmed = false;
    bool verifyExpired = false;
    float verifyBaroAltitude = 0.0f;
    if (xSemaphoreTake(navMutex, portMAX_DELAY) == pdTRUE) {
      if (launchCandidate) {
        nav.beginLaunchVerification(launchDetector.deltaV, raw.timestamp);
      }
      nav.updateIMU(raw);

      if (mode == SystemMode::READY_CONVERGING && nav.isReadyStable()) {
        becameArmed = true;
      }

      if (mode == SystemMode::LAUNCH_VERIFY &&
          millis() - launchDetector.candidateStartMs >= LAUNCH_VERIFY_MS) {
        verifyExpired = true;
        verifyBaroAltitude = nav.getPress().alt;
      }

      if (mode == SystemMode::FLIGHT && nav.isKfReady()) {
        logger.logState(nav.getNominal());
      }
      xSemaphoreGive(navMutex);
    }

    if (mode == SystemMode::FLIGHT) logger.logImu(raw);

    if (launchCandidate) {
      beginLaunchVerification(raw);
    } else if (becameArmed &&
               getSystemMode() == SystemMode::READY_CONVERGING) {
      setSystemMode(SystemMode::ARMED);
      sendResponse("ARMED - LAUNCH DETECTION ACTIVE\n");
      beep(100, 2);
    } else if (verifyExpired &&
               getSystemMode() == SystemMode::LAUNCH_VERIFY) {
      if (verifyBaroAltitude >= LAUNCH_VERIFY_ALT_M) {
        confirmLaunch();
      } else {
        rejectFalseLaunch(raw.timestamp);
      }
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

    SystemMode mode = getSystemMode();
    bool preflightBuffering =
        mode == SystemMode::READY_CONVERGING ||
        mode == SystemMode::ARMED ||
        mode == SystemMode::LAUNCH_VERIFY;
    if (preflightBuffering) preTrigger.pushBaro(p);
    if (mode == SystemMode::FLIGHT) logger.logBaro(p);

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

SystemMode getSystemMode() {
  portENTER_CRITICAL(&modeMux);
  SystemMode mode = systemMode;
  portEXIT_CRITICAL(&modeMux);
  return mode;
}

void setSystemMode(SystemMode mode) {
  portENTER_CRITICAL(&modeMux);
  systemMode = mode;
  portEXIT_CRITICAL(&modeMux);
}

void resetLaunchDetector() {
  launchDetector = LaunchDetectorState{};
}

bool updateLaunchDetector(const Raw_imu &raw) {
  static constexpr float ACCEL_SCALE =
      0.976f * 0.001f * 9.80665f;
  const float accelX = (float)raw.ax * ACCEL_SCALE;
  const float trigger = LAUNCH_ACCEL_THRESHOLD_G * 9.80665f;
  const float reset = LAUNCH_RESET_THRESHOLD_G * 9.80665f;

  if (!launchDetector.candidate) {
    if (accelX < trigger) return false;

    launchDetector.candidate = true;
    launchDetector.firstTimestampUs = raw.timestamp;
    launchDetector.lastTimestampUs = raw.timestamp;
    launchDetector.candidateStartMs = millis();
    launchDetector.belowResetStartMs = 0;
    launchDetector.deltaV = 0.0f;
    return false;
  }

  float dt = (float)(raw.timestamp - launchDetector.lastTimestampUs) * 1e-6f;
  launchDetector.lastTimestampUs = raw.timestamp;
  if (dt > 0.0f && dt < 0.1f) {
    launchDetector.deltaV += fmaxf(accelX - 9.80665f, 0.0f) * dt;
  }

  if (accelX < reset) {
    if (launchDetector.belowResetStartMs == 0) {
      launchDetector.belowResetStartMs = millis();
    } else if (millis() - launchDetector.belowResetStartMs >=
               LAUNCH_RESET_HOLD_MS) {
      resetLaunchDetector();
      return false;
    }
  } else {
    launchDetector.belowResetStartMs = 0;
  }

  return launchDetector.deltaV >= LAUNCH_DV_THRESHOLD_MPS;
}

void beginLaunchVerification(const Raw_imu &raw) {
  xSemaphoreTake(stateMutex, portMAX_DELAY);
  if (getSystemMode() != SystemMode::ARMED) {
    xSemaphoreGive(stateMutex);
    return;
  }

  setSystemMode(SystemMode::LAUNCH_VERIFY);
  xSemaphoreTake(spiMutex, portMAX_DELAY);
  bmp.freezeReference();
  xSemaphoreGive(spiMutex);

  launchStartMs = launchDetector.candidateStartMs;
  xSemaphoreGive(stateMutex);
  sendResponse("LAUNCH CANDIDATE - VERIFYING 1s/5m\n");
}

void confirmLaunch() {
  xSemaphoreTake(stateMutex, portMAX_DELAY);
  if (getSystemMode() != SystemMode::LAUNCH_VERIFY) {
    xSemaphoreGive(stateMutex);
    return;
  }

  setSystemMode(SystemMode::LAUNCH_COMMIT);
  preTrigger.freeze();
  logger.setEnabled(true);

  const size_t count = preTrigger.size();
  PreTriggerBuffer::Record record;
  for (size_t i = 0; i < count; i++) {
    if (!preTrigger.getOldest(i, record)) break;
    if (record.type == PreTriggerBuffer::RECORD_IMU) {
      logger.logImu(record.imu);
    } else if (record.type == PreTriggerBuffer::RECORD_BARO) {
      logger.logBaro(record.baro);
    }
  }
  preTrigger.clear();

  flightActive = true;
  setSystemMode(SystemMode::FLIGHT);
  xSemaphoreGive(stateMutex);
  xTaskNotifyGive(FlightTaskHandle);
}

void rejectFalseLaunch(uint32_t timestamp) {
  xSemaphoreTake(stateMutex, portMAX_DELAY);
  if (getSystemMode() != SystemMode::LAUNCH_VERIFY) {
    xSemaphoreGive(stateMutex);
    return;
  }

  setSystemMode(SystemMode::LAUNCH_COMMIT);
  xSemaphoreTake(navMutex, portMAX_DELAY);
  nav.resumeZuptAfterFalseLaunch(timestamp);
  xSemaphoreGive(navMutex);

  xSemaphoreTake(spiMutex, portMAX_DELAY);
  bmp.startReferenceTracking(READY_BARO_REF_TAU_S);
  xSemaphoreGive(spiMutex);

  preTrigger.clear();
  resetLaunchDetector();
  setSystemMode(SystemMode::READY_CONVERGING);
  xSemaphoreGive(stateMutex);
  sendResponse("FALSE LAUNCH REJECTED - RECONVERGING\n");
}

void disarmSystem() {
  xSemaphoreTake(stateMutex, portMAX_DELAY);
  SystemMode mode = getSystemMode();

  digitalWrite(PYRO_1_PIN, LOW);
  digitalWrite(PYRO_2_PIN, LOW);

  if (mode == SystemMode::FLIGHT) {
    // The command is acknowledged, but an in-flight packet cannot disable
    // navigation, logging, or recovery deployment.
    xSemaphoreGive(stateMutex);
    sendResponse("DISARM ACK - ACTIVE FLIGHT CONTINUES\n");
    return;
  }

  if (logger.isEnabled()) {
    logger.setEnabled(false);
    vTaskDelay(pdMS_TO_TICKS(100));
    logger.forceFlushBuffer();
  }

  xSemaphoreTake(spiMutex, portMAX_DELAY);
  bmp.freezeReference();
  xSemaphoreGive(spiMutex);

  xSemaphoreTake(navMutex, portMAX_DELAY);
  nav.kfReset();
  xSemaphoreGive(navMutex);

  preTrigger.clear();
  resetLaunchDetector();
  flightActive = false;
  setSystemMode(SystemMode::IDLE);
  xSemaphoreGive(stateMutex);
  sendResponse("DISARMED\n");
  beep(100);
}
