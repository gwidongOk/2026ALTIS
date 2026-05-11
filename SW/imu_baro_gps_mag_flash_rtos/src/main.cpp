#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <esp_timer.h>

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

static TaskHandle_t TaskHandle_IMU   = NULL;
static TaskHandle_t TaskHandle_BMP   = NULL;
static TaskHandle_t FlightTaskHandle = NULL;
static TaskHandle_t FlushTaskHandle  = NULL;

static SemaphoreHandle_t spiMutex   = NULL;
static SemaphoreHandle_t navMutex   = NULL;
static SemaphoreHandle_t flashMutex = NULL;

static volatile bool flightActive = false;
static FlightPhase flightPhase = FlightPhase::PRE_FLIGHT;

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
  pinMode(LED_PIN,    OUTPUT);
  pinMode(PYRO_1_PIN, OUTPUT); pinMode(PYRO_2_PIN, OUTPUT);
  pinMode(SERVO_1_PIN, OUTPUT); pinMode(SERVO_2_PIN, OUTPUT);
  pinMode(SERVO_3_PIN, OUTPUT); pinMode(SERVO_4_PIN, OUTPUT);
  
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(LED_PIN,    LOW);
  digitalWrite(PYRO_1_PIN, LOW); digitalWrite(PYRO_2_PIN, LOW);

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
  xTaskCreatePinnedToCore(IMU_Task, "IMU_T",  STACK_SIZE_SENSOR, NULL, TASK_C1_PRIO_IMU, &TaskHandle_IMU, 1);
  xTaskCreatePinnedToCore(BMP_Task, "BMP_T",  STACK_SIZE_SENSOR, NULL, TASK_C0_PRIO_BMP, &TaskHandle_BMP, 0);
  xTaskCreatePinnedToCore(Flight_Task,"Flt_T", 4096,              NULL, 3,                 &FlightTaskHandle, 0);
  xTaskCreatePinnedToCore(FlushTask,"Flush_T",STACK_SIZE_FLUSH,  NULL, TASK_C0_PRIO_FLUSH, &FlushTaskHandle, 0);

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

  // --- Pre-Flight Commands ---
  else if (!flightActive) {
    if (cmd == "CALIBRATE") {
      digitalWrite(LED_PIN, HIGH);
      bool imuOk = false, bmpOk = false;
      while (!(imuOk && bmpOk)) {
        sendResponse("CALIBRATING IMU+BMP...\n");
        imuOk = imu.calibrate(CALIB_SAMPLES);
        bmpOk = bmp.calibrate(CALIB_SAMPLES);
        clearSensors();

        if (imuOk && bmpOk) {
          sendResponse("CALIBRATION DONE.\n");
          digitalWrite(LED_PIN, LOW); beep(200);
        } else {
          if (!imuOk) sendResponse("IMU NOISY - RETRYING...\n");
          if (!bmpOk) sendResponse("BARO UNSTABLE - RETRYING...\n");
          beep(100, 3); 
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
    else if (cmd == "ZUPT") {
      sendResponse("ALIGNING KF...\n");
      if (xSemaphoreTake(navMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
          nav.ekfBegin(); // Simplified: just init with current baro
          xSemaphoreGive(navMutex);
          sendResponse("KF READY\n");
          beep(100);
      }
    }
    else if (cmd == "START") {
      sendResponse("STARTING...\n");
      if (xSemaphoreTake(navMutex, portMAX_DELAY) == pdTRUE) {
        if (!nav.isEkfReady()) {
          nav.ekfBegin();
        }
        xSemaphoreGive(navMutex);
      }
      sendResponse("KF READY\n");

      clearSensors();
      logger.setEnabled(true);
      flightActive = true;
      clearSensors(); 
      digitalWrite(LED_PIN, HIGH);
      beep(300);
      sendResponse("FLIGHT ACTIVE\n");
      xTaskNotifyGive(FlightTaskHandle); // Flight_Task 깨우기
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
        digitalWrite(LED_PIN, LOW);
        beep(200); vTaskDelay(pdMS_TO_TICKS(50)); beep(200);
        sendResponse("STOPPED.\n");
      }
    }
    else if (cmd == "PARSE") {
      bool wasLogging = logger.isEnabled();
      if (wasLogging) {
        logger.setEnabled(false);
        vTaskDelay(pdMS_TO_TICKS(100));
      }
      sendResponse("DUMP START\n");
      logger.forceFlushBuffer();
      logger.dumpRawBinary(Serial);
      sendResponse("DUMP DONE\n");
      
      if (wasLogging) { flightActive = false; digitalWrite(LED_PIN, LOW); }
    }
  }

  vTaskDelay(pdMS_TO_TICKS(50));
}

// ============================================================
// Flight Control Task (Core 0, 10Hz)
// ============================================================
void Flight_Task(void *pvParameters) {
  uint32_t pyro1_start_ms = 0;
  bool pyro1_active = false;

  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    
    flightPhase = FlightPhase::PRE_FLIGHT;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (flightActive) {
      vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100)); // 10Hz

      if (pyro1_active && (millis() - pyro1_start_ms > 1000)) {
        digitalWrite(PYRO_1_PIN, LOW);
        pyro1_active = false;
      }

      State_nominal s;
      if (xSemaphoreTake(navMutex, 0) == pdTRUE) {
        s = nav.getNominal();
        xSemaphoreGive(navMutex);
      } else continue;

      float alt = -s.p[2];
      float vel_up = -s.v[2];
      
      // Vertical acceleration from KF bias-corrected state
      // (Approximate using 1D KF bias)
      float az_corr = nav.getStateImu().az - s.ba[2] - 9.80665f;
      float amag = fabsf(az_corr);

      switch (flightPhase) {
        case FlightPhase::PRE_FLIGHT:
          if (alt > 5.0f || vel_up > 5.0f) {
            flightPhase = FlightPhase::POWERED_FLIGHT;
            logger.logEvent(flightPhase, 1);
            sendResponse("LAUNCH\n");
          }
          break;

        case FlightPhase::POWERED_FLIGHT:
          if (amag < 2.0f && vel_up > 10.0f) { // Simple burnout: low acceleration while still fast
            flightPhase = FlightPhase::COASTING;
            logger.logEvent(flightPhase, 2);
            sendResponse("BO\n");
          }
          break;

        case FlightPhase::COASTING: {
          static uint8_t descent_count = 0;
          if (vel_up < -0.5f && alt > 15.0f) {
            descent_count++;
            if (descent_count >= 3) {
              flightPhase = FlightPhase::DESCENT;
              logger.logEvent(flightPhase, 3);
              sendResponse("APG\n");
              digitalWrite(PYRO_1_PIN, HIGH);
              pyro1_start_ms = millis();
              pyro1_active = true;
              descent_count = 0;
            }
          } else {
            descent_count = 0;
          }
          break;
        }

        case FlightPhase::DESCENT:
          if (alt < 10.0f && fabsf(vel_up) < 1.0f) {
            flightPhase = FlightPhase::LANDED;
            logger.logEvent(flightPhase, 4);
            sendResponse("LAND\n");
          }
          break;

        case FlightPhase::LANDED:
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
      if (flightActive && nav.isEkfReady()) {
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
  imu.readRawIMU(d1, d2, d3, d4, d5, d6);
  bmp.readData(df);
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
