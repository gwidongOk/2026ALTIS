#include <Arduino.h>
#include <SPI.h>
#include <esp_timer.h>

#include "Config.h"
#include "LSM6DSO32.h"
#include "BMP388.h"
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
static MX25Logger logger;
static TaskHandle_t TaskHandle_IMU  = NULL;
static TaskHandle_t TaskHandle_BMP  = NULL;
static TaskHandle_t FlushTaskHandle = NULL;
static SemaphoreHandle_t spiMutex   = NULL;
static SemaphoreHandle_t flashMutex = NULL;

static volatile bool measActive = false;

// Forward Declarations
void IMU_Task(void *pvParameters);
void BMP_Task(void *pvParameters);
void FlushTask(void *pvParameters);
void beep(int ms, int count = 1);
void attachSensorInterrupts();
void detachSensorInterrupts();

// ============================================================
// setup()
// ============================================================
void setup() {
  Serial.begin(SERIAL_BAUD);

  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_PIN,    OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(LED_PIN,    LOW);

  spiMutex   = xSemaphoreCreateMutex();
  flashMutex = xSemaphoreCreateMutex();
  if (!spiMutex || !flashMutex) { beep(100, 3); while (1); }

  initBLE(BLE_DEVICE_NAME);
  sensorSPI.begin(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN);

  bool ok = true;
  if (!imu.begin()) { sendResponse("IMU FAIL\n"); ok = false; }
  if (!bmp.begin()) { sendResponse("BMP FAIL\n"); ok = false; }
  if (!ok) { beep(100, 3); while (1); }
  sendResponse("ALL SENSORS OK\n");

  logger.begin(&flashSPI, FLASH_SCK_PIN, FLASH_MISO_PIN, FLASH_MOSI_PIN,
               FLASH_CS_PIN, flashMutex);

  pinMode(IMU_INT1_PIN, INPUT_PULLDOWN);
  pinMode(BMP_INT_PIN,  INPUT);
  attachSensorInterrupts();
  imu.enableAccelDataReadyInterrupt(1);

  xTaskCreatePinnedToCore(IMU_Task,  "IMU_T",   STACK_SIZE_SENSOR, NULL, TASK_C1_PRIO_IMU,   &TaskHandle_IMU,   1);
  xTaskCreatePinnedToCore(BMP_Task,  "BMP_T",   STACK_SIZE_SENSOR, NULL, TASK_C0_PRIO_BMP,   &TaskHandle_BMP,   0);
  xTaskCreatePinnedToCore(FlushTask, "Flush_T", STACK_SIZE_FLUSH,  NULL, TASK_C0_PRIO_FLUSH, &FlushTaskHandle,  0);

  beep(200);
  sendResponse(">>> READY\n");
}

// ============================================================
// loop() : Command Dispatcher
//
//  CALIBRATE  - IMU+BMP 정적 캘리브레이션 수행
//  START      - Flash에 데이터 기록 시작
//  STOP       - 기록 중단 및 Flash flush
//  PARSE      - Flash 이진 데이터를 Serial로 덤프
//  ERASE      - Flash 전체 삭제
//  REBOOT     - 재부팅
// ============================================================
void loop() {
  String cmd = getIncomingRaw();
  if (cmd.length() == 0) { vTaskDelay(pdMS_TO_TICKS(50)); return; }
  cmd.toUpperCase();

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
  else if (cmd == "CALIBRATE") {
    if (measActive) { sendResponse("STOP MEAS FIRST\n"); return; }

    // 센서 태스크 정지 후 캘리브레이션 (SPI 충돌 방지)
    vTaskSuspend(TaskHandle_IMU);
    vTaskSuspend(TaskHandle_BMP);
    detachSensorInterrupts();
    vTaskDelay(pdMS_TO_TICKS(10));

    digitalWrite(LED_PIN, HIGH);
    bool imuOk = false, bmpOk = false;

    while (!(imuOk && bmpOk)) {
      sendResponse("CALIBRATING IMU+BMP...\n");
      imuOk = imu.calibrate(CALIB_SAMPLES);
      bmpOk = bmp.calibrate(CALIB_SAMPLES);

      if (imuOk && bmpOk) {
        sendResponse("CALIBRATION DONE.\n");
        digitalWrite(LED_PIN, LOW);
        beep(200);
      } else {
        if (!imuOk) sendResponse("IMU NOISY - RETRYING...\n");
        if (!bmpOk) sendResponse("BARO UNSTABLE - RETRYING...\n");
        beep(100, 3);
        vTaskDelay(pdMS_TO_TICKS(1000));
      }
    }

    attachSensorInterrupts();
    imu.enableAccelDataReadyInterrupt(1);
    vTaskResume(TaskHandle_IMU);
    vTaskResume(TaskHandle_BMP);
  }
  else if (cmd == "START") {
    if (measActive) { sendResponse("ALREADY ACTIVE\n"); return; }
    measActive = true;
    logger.setEnabled(true);
    digitalWrite(LED_PIN, HIGH);
    beep(200);
    sendResponse("STARTED\n");
  }
  else if (cmd == "STOP") {
    if (!measActive) { sendResponse("NOT MEASURING\n"); return; }
    logger.setEnabled(false);
    vTaskDelay(pdMS_TO_TICKS(100));
    logger.forceFlushBuffer();
    measActive = false;
    digitalWrite(LED_PIN, LOW);
    beep(200); vTaskDelay(pdMS_TO_TICKS(50)); beep(200);
    sendResponse("STOPPED.\n");
  }
  else if (cmd == "PARSE") {
    bool wasLogging = logger.isEnabled();
    if (wasLogging) {
      logger.setEnabled(false);
      measActive = false;
      vTaskDelay(pdMS_TO_TICKS(100));
    }
    logger.forceFlushBuffer();
    sendResponse("DUMP START\n");
    logger.dumpRawBinary(Serial);
    sendResponse("DUMP DONE\n");
    if (wasLogging) digitalWrite(LED_PIN, LOW);
  }
  else if (cmd == "ERASE") {
    if (measActive) { sendResponse("STOP MEAS FIRST\n"); return; }
    sendResponse("ERASING FLASH...\n");
    logger.eraseAll();
    sendResponse("DONE.\n");
    beep(500);
  }

  vTaskDelay(pdMS_TO_TICKS(50));
}

// ============================================================
// IMU Task (Core 1, interrupt-driven @ 416Hz)
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
    if (measActive) logger.logImu(raw);
  }
}

// ============================================================
// BMP Task (Core 0, interrupt-driven @ 50Hz)
// ============================================================
void BMP_Task(void *pvParameters) {
  Raw_press p;
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    if (xSemaphoreTake(spiMutex, portMAX_DELAY) == pdTRUE) {
      p.timestamp = (uint32_t)(esp_timer_get_time() & 0xFFFFFFFF);
      bmp.readAltitude(p.alt);
      xSemaphoreGive(spiMutex);
    }

    if (measActive) logger.logBaro(p);
  }
}

// ============================================================
// Flush Task (Core 0)
// ============================================================
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
