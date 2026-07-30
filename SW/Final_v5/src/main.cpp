#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <esp_timer.h>
#include <ESP32Servo.h>
#include <atomic>

#include "Config.h"
#include "LSM6DSO32.h"
#include "BMP388.h"
#include "NAV.h"
#include "MX25Logger.h"
#include "sensor_data.h"
#include "BLE.h"

// ============================================================
// Architecture
// ============================================================
// Core 1: one real-time owner for IMU, BARO, NAV, flight state and actuators.
// Core 0: command/BLE/Serial reception, responses and flash queue draining.
//
// The sensor data-ready interrupts only set IMU/BARO flags. The real-time task
// polls those flags and performs every sensor/NAV/flight operation itself.
// Because that state is single-owner, the old SPI, NAV and state mutexes are
// unnecessary. Only the independent flash SPI mutex remains.

static SPIClass sensorSPI(HSPI);
static SPIClass flashSPI(FSPI);

static LSM6DSO32 imu(IMU_CS_PIN, &sensorSPI);
static BMP388    bmp(BMP_CS_PIN, &sensorSPI);
static NAV       nav;
static MX25Logger logger;
static Servo     servoDrogue;

static TaskHandle_t storageTaskHandle  = nullptr;
static SemaphoreHandle_t flashMutex    = nullptr;
static QueueHandle_t commandQueue      = nullptr;
static QueueHandle_t responseQueue     = nullptr;

static constexpr size_t COMMAND_TEXT_LENGTH  = 32;
static constexpr size_t RESPONSE_TEXT_LENGTH = 224;
static constexpr size_t RESPONSE_QUEUE_LENGTH = 32;
static constexpr size_t PRETRIGGER_COMMIT_BATCH_RECORDS = 32;

struct CommandMessage {
  char text[COMMAND_TEXT_LENGTH];
};

struct ResponseMessage {
  char text[RESPONSE_TEXT_LENGTH];
};

enum class SystemMode : uint8_t {
  IDLE = 0,
  READY_CONVERGING,
  ARMED,
  LAUNCH_VERIFY,
  LAUNCH_COMMIT,
  FLIGHT,
  LANDED
};

struct FlightState {
  FlightPhase phase = PRE_FLIGHT;
  uint32_t lastUpdateMs = 0;
  uint32_t poweredStartMs = 0;
  uint32_t coastingStartMs = 0;
  uint32_t pyro1StartMs = 0;
  uint32_t pyro2StartMs = 0;
  bool pyro1Active = false;
  bool pyro2Active = false;
  bool stage2Attempted = false;
  bool stage2Ignited = false;
  bool mainDeployed = false;
  uint8_t descentCount = 0;
  uint8_t landedCount = 0;
  uint8_t separationCount = 0;
};

static SystemMode systemMode = SystemMode::IDLE;
static bool calibrationDone = false;
static bool flashErasedThisBoot = false;
static bool launchCandidate = false;
static uint32_t launchCandidateStartMs = 0;
static FlightState flight;
static volatile bool storagePaused = false;
static volatile bool imuDataReady = false;
static volatile bool baroDataReady = false;
static portMUX_TYPE sensorFlagMux = portMUX_INITIALIZER_UNLOCKED;
static PreTriggerBuffer preTrigger;
static std::atomic<bool> preTriggerCommitPending{false};

// ============================================================
// Forward declarations
// ============================================================
void RealtimeTask(void *pvParameters);
void StorageCommsTask(void *pvParameters);
void processCommand(const String &cmd);
void serviceImu();
void serviceBaro();
void beep(int ms, int count = 1);
void respond(const char *message);
void rebootSystem(const char *message);
bool pollBlockingAbort(const char *abortMessage);
bool runCalibration();
bool runReadySequence();
bool collectAlignmentSamples(bool &aborted);
bool updateFlight();
void confirmLaunch();
void rejectFalseLaunch(uint32_t timestamp);
void displaySensors();
void stopSystem();
void IRAM_ATTR IMUInterruptHandler();
void IRAM_ATTR BMPInterruptHandler();

// ============================================================
// Initialization
// ============================================================
void setup() {
  Serial.begin(SERIAL_BAUD);

  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(PYRO_1_PIN, OUTPUT);
  pinMode(PYRO_2_PIN, OUTPUT);
  pinMode(SERVO_2_PIN, OUTPUT);
  pinMode(SERVO_3_PIN, OUTPUT);
  pinMode(SERVO_4_PIN, OUTPUT);

  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(PYRO_1_PIN, LOW);
  digitalWrite(PYRO_2_PIN, LOW);

  servoDrogue.attach(SERVO_1_PIN);
  servoDrogue.write(SERVO_DROGUE_IDLE_DEG);

  // External 4.7k pull-up:
  // LOW = stages joined, HIGH = separation confirmed.
  pinMode(STAGE_SEP_PIN, INPUT);

  flashMutex = xSemaphoreCreateMutex();
  commandQueue = xQueueCreate(COMMAND_QUEUE_LENGTH, sizeof(CommandMessage));
  responseQueue = xQueueCreate(RESPONSE_QUEUE_LENGTH, sizeof(ResponseMessage));
  if (!flashMutex || !commandQueue || !responseQueue) {
    beep(100, 3);
    while (true) vTaskDelay(portMAX_DELAY);
  }
  if (!preTrigger.begin(PRETRIGGER_RECORD_CAPACITY)) {
    sendResponse("PRETRIGGER PSRAM FAIL\n");
    beep(100, 3);
    while (true) vTaskDelay(portMAX_DELAY);
  }

  initBLE(BLE_DEVICE_NAME);
  sensorSPI.begin(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN);

  bool sensorsOk = true;
  if (!imu.begin()) {
    sendResponse("IMU FAIL\n");
    sensorsOk = false;
  }
  if (!bmp.begin()) {
    sendResponse("BMP FAIL\n");
    sensorsOk = false;
  }
  if (!sensorsOk) {
    beep(100, 3);
    while (true) vTaskDelay(portMAX_DELAY);
  }
  sendResponse("IMU+BARO OK\n");

  if (!logger.begin(&flashSPI, FLASH_SCK_PIN, FLASH_MISO_PIN,
                    FLASH_MOSI_PIN, FLASH_CS_PIN, flashMutex)) {
    sendResponse("LOGGER INIT FAIL\n");
    beep(100, 3);
    while (true) vTaskDelay(portMAX_DELAY);
  }
  BaseType_t realtimeCreated = xTaskCreatePinnedToCore(
      RealtimeTask, "Realtime", STACK_SIZE_REALTIME, nullptr,
      TASK_C1_PRIO_REALTIME, nullptr, 1);
  BaseType_t storageCreated = xTaskCreatePinnedToCore(
      StorageCommsTask, "StorageComms", STACK_SIZE_STORAGE, nullptr,
      TASK_C0_PRIO_STORAGE, &storageTaskHandle, 0);

  if (realtimeCreated != pdPASS || storageCreated != pdPASS) {
    sendResponse("TASK CREATE FAIL\n");
    beep(100, 3);
    while (true) vTaskDelay(portMAX_DELAY);
  }

  pinMode(IMU_INT1_PIN, INPUT_PULLDOWN);
  pinMode(BMP_INT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(IMU_INT1_PIN),
                  IMUInterruptHandler, RISING);
  attachInterrupt(digitalPinToInterrupt(BMP_INT_PIN),
                  BMPInterruptHandler, RISING);
  imu.enableAccelDataReadyInterrupt(1);

  beep(200);
  sendResponse(">>> V5 TWO-TASK READY\n");
}

void loop() {
  // Arduino's loop task is intentionally unused. All work is owned by the two
  // explicit tasks above.
  vTaskDelay(pdMS_TO_TICKS(1000));
}

// ============================================================
// Core 1: single real-time task
// ============================================================
void RealtimeTask(void *pvParameters) {
  (void)pvParameters;

  for (;;) {
    bool imuReady = false;
    bool baroReady = false;
    portENTER_CRITICAL(&sensorFlagMux);
    imuReady = imuDataReady;
    baroReady = baroDataReady;
    imuDataReady = false;
    baroDataReady = false;
    portEXIT_CRITICAL(&sensorFlagMux);

    if (imuReady) {
      serviceImu();
    }
    if (baroReady) {
      serviceBaro();
    }

    // Commands are executed by the state owner. Handle one per pass so a
    // burst of pre-flight commands cannot starve sensor servicing.
    CommandMessage incoming = {};
    if (xQueueReceive(commandQueue, &incoming, 0) == pdTRUE) {
      processCommand(String(incoming.text));
    }

    // A single 3 g sample starts verification. Nothing is written to flash
    // until the barometric altitude confirms the launch one second later.
    if (launchCandidate &&
        systemMode == SystemMode::LAUNCH_VERIFY &&
        millis() - launchCandidateStartMs >= LAUNCH_VERIFY_MS) {
      if (nav.getPress().alt > LAUNCH_VERIFY_ALT_M) {
        confirmLaunch();
      } else {
        rejectFalseLaunch(nav.getRawImu().timestamp);
      }
    }

    if (systemMode == SystemMode::FLIGHT && updateFlight()) {
      logger.setEnabled(false);
      vTaskDelay(pdMS_TO_TICKS(100));
      storagePaused = true;
      vTaskDelay(pdMS_TO_TICKS(20));
      logger.forceFlushBuffer();
      storagePaused = false;
      systemMode = SystemMode::LANDED;
      beep(500, 3);
      respond("STOPPED.\n");
    }

    // The task only polls data-ready flags; yield for one RTOS tick so the
    // Core 1 idle/Arduino tasks can still run and feed the watchdog.
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// ============================================================
// Core 0: communication and storage task
// ============================================================
void StorageCommsTask(void *pvParameters) {
  (void)pvParameters;
  size_t preTriggerCommitIndex = 0;

  for (;;) {
    // Responses are emitted here so BLE/Serial work never delays a sensor
    // update or an actuator timer on the real-time core.
    ResponseMessage response = {};
    for (int i = 0;
         i < 4 && xQueueReceive(responseQueue, &response, 0) == pdTRUE;
         i++) {
      sendResponse(response.text);
    }

    String cmd = getIncomingRaw();
    if (cmd.length() > 0) {
      cmd.trim();
      cmd.toUpperCase();

      CommandMessage message = {};
      cmd.toCharArray(message.text, sizeof(message.text));
      if (xQueueSend(commandQueue, &message, 0) != pdTRUE) {
        sendResponse("CMD QUEUE FULL\n");
      }
    }

    const bool commitPending =
        preTriggerCommitPending.load(std::memory_order_acquire);
    if (!commitPending) {
      preTriggerCommitIndex = 0;
    }

    if (!storagePaused && commitPending) {
      const size_t count = preTrigger.size();
      if (preTriggerCommitIndex >= count) {
        preTrigger.clear();
        preTriggerCommitIndex = 0;
        preTriggerCommitPending.store(false, std::memory_order_release);
      } else {
        const size_t remaining = count - preTriggerCommitIndex;
        const size_t batch =
            remaining < PRETRIGGER_COMMIT_BATCH_RECORDS
                ? remaining
                : PRETRIGGER_COMMIT_BATCH_RECORDS;
        const size_t end = preTriggerCommitIndex + batch;
        PreTriggerBuffer::Record record;

        while (preTriggerCommitIndex < end) {
          if (!preTrigger.getOldest(preTriggerCommitIndex, record)) break;
          logger.appendPreTriggerRecord(record);
          preTriggerCommitIndex++;
        }
        logger.flushPages();

        if (preTriggerCommitIndex >= count) {
          preTrigger.clear();
          preTriggerCommitIndex = 0;
          preTriggerCommitPending.store(false, std::memory_order_release);
        }
      }
    } else if (!storagePaused) {
      logger.serviceFlush();
    }

    // serviceFlush is non-blocking so a launch commit takes priority before
    // any newer live packet is removed from the queue.
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// ============================================================
// Command dispatcher (runs only in RealtimeTask)
// ============================================================
void processCommand(const String &cmd) {
  SystemMode mode = systemMode;

  if (cmd == "REBOOT") {
    rebootSystem("REBOOTING...\n");
    return;
  }

  if (cmd == "PARSE") {
    if (mode != SystemMode::IDLE && mode != SystemMode::LANDED) {
      respond("PARSE BLOCKED - STOP OR LAND FIRST\n");
      return;
    }

    respond("DUMP START\n");
    storagePaused = true;
    vTaskDelay(pdMS_TO_TICKS(20));
    logger.forceFlushBuffer();
    logger.dumpRawBinary(Serial);
    storagePaused = false;
    respond("DUMP DONE\n");
    return;
  }

  if (cmd == "DISPLAY") {
    displaySensors();
    return;
  }

  if (cmd == "STOP") {
    stopSystem();
    return;
  }

  if (mode == SystemMode::FLIGHT) {
    respond("FLIGHT ACTIVE - ONLY STOP OR REBOOT\n");
    return;
  }

  bool armSequenceActive =
      mode == SystemMode::READY_CONVERGING ||
      mode == SystemMode::ARMED ||
      mode == SystemMode::LAUNCH_VERIFY ||
      mode == SystemMode::LAUNCH_COMMIT;

  if (armSequenceActive) {
    if (cmd == "READY") {
      char status[64];
      snprintf(status, sizeof(status), "READY MODE %u\n", (unsigned)mode);
      respond(status);
    } else {
      respond("READY ACTIVE - ONLY STOP, DISPLAY OR REBOOT\n");
    }
    return;
  }

  if (cmd == "CALIBRATE") {
    runCalibration();
    return;
  }

  if (cmd == "ERASE") {
    respond("ERASING FLASH...\n");
    storagePaused = true;
    vTaskDelay(pdMS_TO_TICKS(20));
    logger.eraseAll();
    storagePaused = false;
    flashErasedThisBoot = true;
    respond("DONE.\n");
    beep(500);
    return;
  }

  if (cmd == "TEST SERVO") {
    respond("SERVO: IDLE -> DEPLOY -> IDLE\n");
    servoDrogue.write(SERVO_DROGUE_DEPLOY_DEG);
    vTaskDelay(pdMS_TO_TICKS(1000));
    servoDrogue.write(SERVO_DROGUE_IDLE_DEG);
    respond("SERVO OK\n");
    beep(100, 2);
    return;
  }

  if (cmd == "TEST PYRO1") {
    respond("PYRO1: 1000ms pulse (e-match disconnected?)\n");
    beep(50, 3);
    vTaskDelay(pdMS_TO_TICKS(1000));
    digitalWrite(PYRO_1_PIN, HIGH);
    vTaskDelay(pdMS_TO_TICKS(STAGE2_PULSE_MS));
    digitalWrite(PYRO_1_PIN, LOW);
    respond("PYRO1 DONE\n");
    beep(200);
    return;
  }

  if (cmd == "TEST PYRO2") {
    respond("PYRO2: 1000ms pulse (e-match disconnected?)\n");
    beep(50, 3);
    vTaskDelay(pdMS_TO_TICKS(1000));
    digitalWrite(PYRO_2_PIN, HIGH);
    vTaskDelay(pdMS_TO_TICKS(MAIN_PULSE_MS));
    digitalWrite(PYRO_2_PIN, LOW);
    respond("PYRO2 DONE\n");
    beep(200);
    return;
  }

  if (cmd == "READY") {
    runReadySequence();
    return;
  }

  respond("UNKNOWN COMMAND\n");
}

// ============================================================
// Pre-flight procedures
// ============================================================
bool runCalibration() {
  calibrationDone = false;
  bool aborted = false;

  respond("IMU WARMUP (5s)...\n");
  uint32_t warmupStartMs = millis();
  while (millis() - warmupStartMs < IMU_CALIB_WARMUP_MS) {
    if (pollBlockingAbort("CALIBRATION ABORTED\n")) {
      aborted = true;
      break;
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }

  uint8_t stableWindows = 0;
  float sumMeanG[3] = {};
  float sumMeanA[3] = {};
  uint32_t calibrationStartMs = millis();

  while (!aborted &&
         stableWindows < IMU_CALIB_STABLE_WINDOWS &&
         millis() - calibrationStartMs < IMU_CALIB_TIMEOUT_MS) {
    respond("CAL WINDOW (512 samples)...\n");

    bool windowOk = imu.calibrate(IMU_CALIB_WINDOW_SAMPLES, false);
    const LSM6DSO32::CalibStats &stats = imu.getLastCalibStats();

    char line[RESPONSE_TEXT_LENGTH];
    snprintf(line, sizeof(line),
             "GYRO ROBUST SIGMA: %.2f %.2f %.2f (lim %.1f)\n",
             stats.std_g[0], stats.std_g[1], stats.std_g[2],
             LSM6DSO32::MAX_GYRO_STD_LSB);
    respond(line);
    snprintf(line, sizeof(line),
             "ACCEL ROBUST SIGMA: %.2f %.2f %.2f (lim %.1f)\n",
             stats.std_a[0], stats.std_a[1], stats.std_a[2],
             LSM6DSO32::MAX_ACCEL_STD_LSB);
    respond(line);
    snprintf(line, sizeof(line),
             "ACCEL MEAN XYZ: %.1f %.1f %.1f - POSE %s\n",
             stats.mean_a[0], stats.mean_a[1], stats.mean_a[2],
             imu.wasLastPoseValid() ? "OK" : "BAD");
    respond(line);

    if (windowOk) {
      for (int axis = 0; axis < 3; axis++) {
        sumMeanG[axis] += stats.mean_g[axis];
        sumMeanA[axis] += stats.mean_a[axis];
      }
      stableWindows++;
      snprintf(line, sizeof(line), "CAL STABLE %u/%u\n",
               stableWindows, IMU_CALIB_STABLE_WINDOWS);
      respond(line);
    } else {
      stableWindows = 0;
      for (int axis = 0; axis < 3; axis++) {
        sumMeanG[axis] = 0.0f;
        sumMeanA[axis] = 0.0f;
      }
      respond(imu.wasLastPoseValid()
                  ? "CAL UNSTABLE - RETRY WINDOW\n"
                  : "CAL POSE BAD - KEEP VERTICAL\n");
    }

    if (stableWindows >= IMU_CALIB_STABLE_WINDOWS) {
      LSM6DSO32::CalibStats finalStats = stats;
      for (int axis = 0; axis < 3; axis++) {
        finalStats.mean_g[axis] =
            sumMeanG[axis] / (float)IMU_CALIB_STABLE_WINDOWS;
        finalStats.mean_a[axis] =
            sumMeanA[axis] / (float)IMU_CALIB_STABLE_WINDOWS;
      }
      imu.applyCalibration(finalStats);
      calibrationDone = true;
      respond("CALIBRATION DONE.\n");
      beep(200);
      break;
    }

    if (pollBlockingAbort("CALIBRATION ABORTED\n")) {
      aborted = true;
    }
  }

  if (!calibrationDone && !aborted) {
    respond("CALIBRATION TIMEOUT - CHECK MOUNT/POSE\n");
    beep(100, 3);
  }
  return calibrationDone;
}

bool runReadySequence() {
  if (!calibrationDone) {
    respond("CALIBRATE REQUIRED\n");
    return false;
  }
  if (!flashErasedThisBoot) {
    respond("ERASE REQUIRED\n");
    return false;
  }
  if (digitalRead(STAGE_SEP_PIN) == HIGH) {
    respond("STAGE SEP SENSOR NOT LOW - CHECK WIRING/MAGNET\n");
    return false;
  }

  // Start from a clean estimator even when READY is entered after a landing.
  nav.kfReset();

  bool aligned = false;
  bool aborted = false;
  while (!aligned && !aborted) {
    respond("READY ALIGNING...\n");
    aligned = collectAlignmentSamples(aborted);
    if (!aligned && !aborted) {
      respond("KF ALIGN RETRY\n");
      beep(100, 3);
      if (pollBlockingAbort("READY ABORTED\n")) {
        aborted = true;
      } else {
        vTaskDelay(pdMS_TO_TICKS(500));
      }
    }
  }
  if (aborted) return false;

  bool baroOk = false;
  Raw_press zeroPress = {};
  while (!baroOk && !aborted) {
    respond("READY BARO ZEROING...\n");
    baroOk = bmp.calibrate(BARO_ZERO_SAMPLES);
    if (baroOk) {
      zeroPress.timestamp =
          (uint32_t)(esp_timer_get_time() & 0xFFFFFFFFULL);
      baroOk = bmp.readAltitude(zeroPress.alt);
      if (baroOk) {
        bmp.startReferenceTracking(READY_BARO_REF_TAU_S);
      }
    }

    if (!baroOk) {
      respond("BARO UNSTABLE - RETRYING...\n");
      beep(100, 3);
      if (pollBlockingAbort("READY ABORTED\n")) {
        aborted = true;
      } else {
        vTaskDelay(pdMS_TO_TICKS(1000));
      }
    }
  }
  if (aborted) return false;

  nav.updatePress(zeroPress);
  nav.beginZupt();
  preTrigger.clear();
  preTriggerCommitPending.store(false, std::memory_order_release);
  launchCandidate = false;
  launchCandidateStartMs = 0;

  // Finish feedback before enabling convergence/launch processing.
  beep(200, 2);
  systemMode = SystemMode::READY_CONVERGING;
  respond("READY CONVERGING - WAIT FOR ARMED\n");
  return true;
}

bool collectAlignmentSamples(bool &aborted) {
  static constexpr int ALIGN_SAMPLE_COUNT = 400;
  Eigen::Vector3f accelSum = Eigen::Vector3f::Zero();

  for (int sample = 0; sample < ALIGN_SAMPLE_COUNT; sample++) {
    Raw_imu raw = {};
    raw.timestamp =
        (uint32_t)(esp_timer_get_time() & 0xFFFFFFFFULL);
    imu.readCalibratedIMU(raw.gx, raw.gy, raw.gz,
                          raw.ax, raw.ay, raw.az);
    nav.updateIMU(raw);

    State_imu state = nav.getStateImu();
    accelSum += Eigen::Vector3f(state.ax, state.ay, state.az);

    if ((sample & 0x0F) == 0 &&
        pollBlockingAbort("READY ABORTED\n")) {
      aborted = true;
      return false;
    }

    // IMU ODR is 416 Hz. A 3 ms interval gives distinct fresh samples while
    // leaving time for the command task to enqueue STOP/REBOOT.
    vTaskDelay(pdMS_TO_TICKS(3));
  }

  Eigen::Vector3f average = accelSum / (float)ALIGN_SAMPLE_COUNT;
  return nav.kfBeginFromAccelAverage(
      average.x(), average.y(), average.z());
}

bool pollBlockingAbort(const char *abortMessage) {
  CommandMessage message = {};
  while (xQueueReceive(commandQueue, &message, 0) == pdTRUE) {
    String cmd(message.text);
    if (cmd == "STOP") {
      respond(abortMessage);
      stopSystem();
      return true;
    }
    if (cmd == "REBOOT") {
      rebootSystem("READY/CALIBRATION ABORTED - REBOOTING...\n");
      return true;
    }
    respond("BUSY - ONLY STOP OR REBOOT\n");
  }
  return false;
}

// ============================================================
// Sensor servicing (RealtimeTask only)
// ============================================================
void serviceImu() {
  Raw_imu raw = {};
  raw.timestamp =
      (uint32_t)(esp_timer_get_time() & 0xFFFFFFFFULL);
  imu.readCalibratedIMU(raw.gx, raw.gy, raw.gz,
                        raw.ax, raw.ay, raw.az);

  SystemMode mode = systemMode;
  bool preflightBuffering =
      mode == SystemMode::READY_CONVERGING ||
      mode == SystemMode::ARMED ||
      mode == SystemMode::LAUNCH_VERIFY;
  if (preflightBuffering) {
    preTrigger.pushImu(raw);
  }

  static constexpr float ACCEL_SCALE =
      0.976f * 0.001f * 9.80665f;
  if (mode == SystemMode::ARMED &&
      (float)raw.ax * ACCEL_SCALE >=
          LAUNCH_ACCEL_THRESHOLD_G * 9.80665f) {
    launchCandidate = true;
    launchCandidateStartMs = millis();
    // Store the launch event beside the exact IMU sample that crossed 3 g.
    // A false launch clears this RAM-only record; a confirmed launch commits it.
    preTrigger.pushEvent(POWERED_FLIGHT, 1, raw.timestamp);
    // Reset at the detected liftoff sample, not one second later after the
    // rocket has already rotated: attitude returns to the saved rail
    // reference and position/velocity/KF return to zero.
    nav.beginLaunchVerification(0.0f, raw.timestamp);
    systemMode = SystemMode::LAUNCH_VERIFY;
    bmp.freezeReference();
    respond("LAUNCH CANDIDATE - VERIFYING 1s/10m\n");
  }

  nav.updateIMU(raw);
  if (preflightBuffering && nav.isKfReady()) {
    preTrigger.pushState(nav.getNominal());
  }

  if (mode == SystemMode::FLIGHT) {
    logger.logImu(raw);
    if (nav.isKfReady()) {
      logger.logState(nav.getNominal());
    }
  }

  if (mode == SystemMode::READY_CONVERGING && nav.isReadyStable()) {
    // Do not activate launch detection until the acknowledgement beeps end.
    beep(100, 2);
    systemMode = SystemMode::ARMED;
    respond("ARMED - LAUNCH DETECTION ACTIVE\n");
  }
}

void serviceBaro() {
  Raw_press press = {};
  press.timestamp =
      (uint32_t)(esp_timer_get_time() & 0xFFFFFFFFULL);
  if (!bmp.readAltitude(press.alt)) {
    return;
  }

  SystemMode mode = systemMode;
  bool preflightBuffering =
      mode == SystemMode::READY_CONVERGING ||
      mode == SystemMode::ARMED ||
      mode == SystemMode::LAUNCH_VERIFY;
  if (preflightBuffering) {
    preTrigger.pushBaro(press);
  }
  if (mode == SystemMode::FLIGHT) {
    logger.logBaro(press);
  }

  nav.updatePress(press);
}

// ============================================================
// Flight state machine
// ============================================================
bool updateFlight() {
  const uint32_t nowMs = millis();

  if (flight.pyro1Active &&
      nowMs - flight.pyro1StartMs >= STAGE2_PULSE_MS) {
    digitalWrite(PYRO_1_PIN, LOW);
    flight.pyro1Active = false;
  }
  if (flight.pyro2Active &&
      nowMs - flight.pyro2StartMs >= MAIN_PULSE_MS) {
    digitalWrite(PYRO_2_PIN, LOW);
    flight.pyro2Active = false;
  }

  if (nowMs - flight.lastUpdateMs < 100) return false;
  flight.lastUpdateMs = nowMs;

  const State_nominal nominal = nav.getNominal();
  const State_imu imuState = nav.getStateImu();
  const float altitude = -nominal.p[2];
  const float velocityUp = -nominal.v[2];
  const float force = sqrtf(
      imuState.ax * imuState.ax +
      imuState.ay * imuState.ay +
      imuState.az * imuState.az);

  switch (flight.phase) {
    case PRE_FLIGHT:
      break;

    case POWERED_FLIGHT: {
      const bool burnout = force < 2.0f && velocityUp > 10.0f;
      const bool timeout =
          flight.poweredStartMs != 0 &&
          nowMs - flight.poweredStartMs >= BURNOUT_TIMEOUT_MS;
      if (burnout || timeout) {
        flight.phase = COASTING;
        flight.coastingStartMs = nowMs;
        logger.logEvent(flight.phase, 2);
        respond(timeout && !burnout ? "BO TIMEOUT\n" : "BO\n");
      }
      break;
    }

    case COASTING: {
      if (!flight.stage2Attempted) {
        flight.separationCount =
            digitalRead(STAGE_SEP_PIN) == HIGH
                ? flight.separationCount + 1
                : 0;

        if (flight.separationCount >= 3) {
          flight.stage2Attempted = true;
          const float cosTilt =
              2.0f * (nominal.q[0] * nominal.q[2] -
                      nominal.q[1] * nominal.q[3]);
          const bool tiltOk =
              cosTilt > cosf(TILT_LIMIT_DEG *
                              (float)M_PI / 180.0f);

          if (tiltOk) {
            digitalWrite(PYRO_1_PIN, HIGH);
            flight.pyro1StartMs = nowMs;
            flight.pyro1Active = true;
            flight.stage2Ignited = true;
            logger.logEvent(flight.phase, 6);
            respond("STAGE2 IGN\n");
          } else {
            logger.logEvent(flight.phase, 5);
            respond("NSC TILT - STAGE2 ABORT\n");
          }
        }
      }

      const bool noIgnitionTimeout =
          !flight.stage2Ignited &&
          flight.coastingStartMs != 0 &&
          nowMs - flight.coastingStartMs >=
              STAGE2_IGNITION_TIMEOUT_MS;
      if (noIgnitionTimeout && !flight.stage2Attempted) {
        flight.stage2Attempted = true;
        logger.logEvent(flight.phase, 5);
        respond("NSC SEP TIMEOUT\n");
      }

      const bool sensorApogee =
          velocityUp < -0.5f && altitude > 15.0f;
      const bool timeoutApogee =
          flight.stage2Ignited &&
          nowMs - flight.pyro1StartMs >= STAGE2_APOGEE_TIMEOUT_MS;

      flight.descentCount =
          sensorApogee ? flight.descentCount + 1 : 0;
      const bool sensorApogeeConfirmed = flight.descentCount >= 3;
      if (!sensorApogeeConfirmed &&
          !noIgnitionTimeout &&
          !timeoutApogee) break;

      if (!flight.stage2Attempted) {
        logger.logEvent(flight.phase, 5);
        respond("NSC NOSEP\n");
      }
      flight.phase = DESCENT;
      logger.logEvent(flight.phase, 3);
      if (sensorApogeeConfirmed) {
        respond("APG\n");
      } else if (noIgnitionTimeout) {
        respond("APG NO IGN TIMEOUT\n");
      } else {
        respond(timeoutApogee ? "APG TIMEOUT\n" : "APG\n");
      }
      servoDrogue.write(SERVO_DROGUE_DEPLOY_DEG);
      respond("DROGUE\n");
      flight.descentCount = 0;
      break;
    }

    case DESCENT:
      if (!flight.mainDeployed && altitude < MAIN_DEPLOY_ALT_M) {
        flight.mainDeployed = true;
        digitalWrite(PYRO_2_PIN, HIGH);
        flight.pyro2StartMs = nowMs;
        flight.pyro2Active = true;
        logger.logEvent(flight.phase, 7);
        respond("MAIN\n");
      }

      if (altitude < 10.0f && fabsf(velocityUp) < 1.0f) {
        if (++flight.landedCount >= 10) {
          flight.phase = LANDED;
          logger.logEvent(flight.phase, 4);
          respond("LAND\n");
          flight.pyro1Active = false;
          flight.pyro2Active = false;
          digitalWrite(PYRO_1_PIN, LOW);
          digitalWrite(PYRO_2_PIN, LOW);
          return true;
        }
      } else {
        flight.landedCount = 0;
      }
      break;

    case LANDED:
      break;
  }

  return false;
}

// ============================================================
// Launch state machine
// ============================================================
void confirmLaunch() {
  if (systemMode != SystemMode::LAUNCH_VERIFY) return;

  const uint32_t poweredStartMs = launchCandidateStartMs;
  launchCandidate = false;
  launchCandidateStartMs = 0;
  systemMode = SystemMode::LAUNCH_COMMIT;
  preTrigger.freeze();

  // Make the frozen backlog visible to Core 0 before enabling live logging.
  // Core 0 writes it in bounded batches while Core 1 immediately continues
  // NAV and flight control. At 1,600 records this avoids an estimated
  // 5-20 ms Core-1 queue-copy stall on ESP32-S3.
  preTriggerCommitPending.store(true, std::memory_order_release);
  logger.setEnabled(true);

  flight = FlightState{};
  flight.phase = POWERED_FLIGHT;
  flight.poweredStartMs = poweredStartMs;
  flight.lastUpdateMs = millis();
  systemMode = SystemMode::FLIGHT;
  respond("LAUNCH CONFIRMED\n");
}

void rejectFalseLaunch(uint32_t timestamp) {
  if (systemMode != SystemMode::LAUNCH_VERIFY) return;

  systemMode = SystemMode::LAUNCH_COMMIT;
  nav.resumeZuptAfterFalseLaunch(timestamp);
  bmp.startReferenceTracking(READY_BARO_REF_TAU_S);

  preTrigger.clear();
  launchCandidate = false;
  launchCandidateStartMs = 0;
  systemMode = SystemMode::READY_CONVERGING;
  respond("FALSE LAUNCH REJECTED - RECONVERGING\n");
}

// ============================================================
// Commands and utilities
// ============================================================
void displaySensors() {
  bool preLaunch =
      systemMode == SystemMode::IDLE ||
      systemMode == SystemMode::READY_CONVERGING ||
      systemMode == SystemMode::ARMED;
  if (!preLaunch) {
    respond("DISPLAY BLOCKED - PRE-LAUNCH ONLY\n");
    return;
  }

  Raw_imu raw = nav.getRawImu();
  State_imu state = nav.getStateImu();
  Raw_press press = nav.getPress();

  char line[RESPONSE_TEXT_LENGTH];
  snprintf(line, sizeof(line),
           "IMU RAW G[%d,%d,%d] A[%d,%d,%d]\n",
           raw.gx, raw.gy, raw.gz, raw.ax, raw.ay, raw.az);
  respond(line);

  snprintf(line, sizeof(line),
           "IMU SI A[%.2f,%.2f,%.2f]m/s2 G[%.2f,%.2f,%.2f]dps\n",
           state.ax, state.ay, state.az,
           state.gx * 180.0f / (float)M_PI,
           state.gy * 180.0f / (float)M_PI,
           state.gz * 180.0f / (float)M_PI);
  respond(line);

  snprintf(line, sizeof(line), "BARO ALT %.2f m\n", press.alt);
  respond(line);

  bool separated = digitalRead(STAGE_SEP_PIN) == HIGH;
  snprintf(line, sizeof(line), "STAGE %s - %s\n",
           separated ? "HIGH" : "LOW",
           separated ? "SEPARATED" : "JOINED");
  respond(line);
}

void stopSystem() {
  SystemMode previousMode = systemMode;

  flight.pyro1Active = false;
  flight.pyro2Active = false;
  digitalWrite(PYRO_1_PIN, LOW);
  digitalWrite(PYRO_2_PIN, LOW);

  if (logger.isEnabled()) {
    logger.setEnabled(false);
    vTaskDelay(pdMS_TO_TICKS(100));
    storagePaused = true;
    vTaskDelay(pdMS_TO_TICKS(20));
    logger.forceFlushBuffer();
    storagePaused = false;
  }

  bmp.freezeReference();
  nav.kfReset();
  preTriggerCommitPending.store(false, std::memory_order_release);
  preTrigger.clear();
  launchCandidate = false;
  launchCandidateStartMs = 0;
  systemMode =
      previousMode == SystemMode::FLIGHT
          ? SystemMode::LANDED
          : SystemMode::IDLE;

  respond("STOPPED.\n");
  beep(200);
  vTaskDelay(pdMS_TO_TICKS(50));
  beep(200);
}

void rebootSystem(const char *message) {
  flight.pyro1Active = false;
  flight.pyro2Active = false;
  digitalWrite(PYRO_1_PIN, LOW);
  digitalWrite(PYRO_2_PIN, LOW);
  preTriggerCommitPending.store(false, std::memory_order_release);
  launchCandidate = false;
  launchCandidateStartMs = 0;
  respond(message);

  if (logger.isEnabled()) {
    logger.setEnabled(false);
    vTaskDelay(pdMS_TO_TICKS(100));
    storagePaused = true;
    vTaskDelay(pdMS_TO_TICKS(20));
    logger.forceFlushBuffer();
    storagePaused = false;
  }

  vTaskDelay(pdMS_TO_TICKS(200));
  ESP.restart();
}

void respond(const char *message) {
  if (!message) return;

  // Before StorageCommsTask exists (boot diagnostics), send directly.
  if (!responseQueue || !storageTaskHandle) {
    sendResponse(message);
    return;
  }

  ResponseMessage response = {};
  strncpy(response.text, message, sizeof(response.text) - 1);
  if (xQueueSend(responseQueue, &response, 0) != pdTRUE) {
    Serial.print(message);
  }
}

void beep(int ms, int count) {
  for (int index = 0; index < count; index++) {
    digitalWrite(BUZZER_PIN, HIGH);
    vTaskDelay(pdMS_TO_TICKS(ms));
    digitalWrite(BUZZER_PIN, LOW);
    if (index < count - 1) {
      vTaskDelay(pdMS_TO_TICKS(ms));
    }
  }
}

// ============================================================
// Sensor ISRs
// ============================================================
void IRAM_ATTR IMUInterruptHandler() {
  portENTER_CRITICAL_ISR(&sensorFlagMux);
  imuDataReady = true;
  portEXIT_CRITICAL_ISR(&sensorFlagMux);
}

void IRAM_ATTR BMPInterruptHandler() {
  portENTER_CRITICAL_ISR(&sensorFlagMux);
  baroDataReady = true;
  portEXIT_CRITICAL_ISR(&sensorFlagMux);
}
