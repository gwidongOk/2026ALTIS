#include <Arduino.h>
#include <esp_timer.h>
#include "Config.h"
#include "BLE.h"

// ============================================================
// 전역 변수 및 객체 실제 메모리 할당
// ============================================================
SPIClass sensorSPI(HSPI);
SPIClass flashSPI(FSPI);

LSM6DSO32 imu(IMU_CS_PIN, &sensorSPI);
BMP388 bmp(BMP_CS_PIN, &sensorSPI);
NAV nav;
MX25Logger logger;

TaskHandle_t TaskHandle_IMU = NULL;
TaskHandle_t TaskHandle_BMP = NULL;
TaskHandle_t TaskHandle_NAV = NULL;
TaskHandle_t FlushTaskHandle = NULL;

SemaphoreHandle_t spiMutex;
SemaphoreHandle_t dataMutex;
SemaphoreHandle_t flashMutex;   // STOP/PARSE/ERASE 명령 시에만 사용
QueueHandle_t logQueue;

volatile bool isLogging = false;
volatile bool systemStarted = false;

// ============================================================
// ISR 및 유틸리티
// ============================================================
inline int64_t getTimeUs() { return esp_timer_get_time(); }
inline uint32_t getTimeUs32() { return (uint32_t)(esp_timer_get_time() & 0xFFFFFFFF); }

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

// ============================================================
// 인터럽트 연결/해제 헬퍼
// ============================================================
void attachSensorInterrupts() {
  attachInterrupt(digitalPinToInterrupt(IMU_INT1_PIN), IMUInterruptHandler, RISING);
  attachInterrupt(digitalPinToInterrupt(BMP_INT_PIN), BMPInterruptHandler, RISING);
}

void detachSensorInterrupts() {
  detachInterrupt(digitalPinToInterrupt(IMU_INT1_PIN));
  detachInterrupt(digitalPinToInterrupt(BMP_INT_PIN));
}

// ============================================================
// 캘리브레이션
// ============================================================
void performCalibration() {
  bool wasLogging = isLogging;
  isLogging = false;

  // 인터럽트 해제 → 센서 태스크가 ulTaskNotifyTake에서 자연 대기
  detachSensorInterrupts();
  vTaskDelay(pdMS_TO_TICKS(100));

  float c_gx, c_gy, c_gz, c_ax, c_ay, c_az, c_p;

  if (xSemaphoreTake(spiMutex, portMAX_DELAY) == pdTRUE) {
    imu.calibrate(c_gx, c_gy, c_gz, c_ax, c_ay, c_az);
    bmp.calibrate(c_p);
    xSemaphoreGive(spiMutex);
  }

  if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
    nav.calibrate(c_gx, c_gy, c_gz, c_ax, c_ay, c_az, c_p);
    xSemaphoreGive(dataMutex);
  }

  // 잔여 데이터 클리어
  if (xSemaphoreTake(spiMutex, portMAX_DELAY) == pdTRUE) {
    int16_t d1, d2, d3, d4, d5, d6; float d0;
    imu.readRawIMU(d1, d2, d3, d4, d5, d6);
    bmp.readData(d0);
    xSemaphoreGive(spiMutex);
  }

  // 인터럽트는 항상 복구 (wasLogging 무관)
  attachSensorInterrupts();

  if (wasLogging) {
    isLogging = true;
  }
}

// ============================================================
// 명령 처리 태스크
// ============================================================
void processCommandTask(void *pvParameters) {
  for (;;) {
    String cmd = getIncomingRaw();
    if (cmd != "") {
      cmd.toUpperCase();

      if (cmd == "START") {
        if (!systemStarted) {
          systemStarted = true;
          sendResponse("SYSTEM STARTING...\n");
        }
        else if (!isLogging) {
          isLogging = true;
          attachSensorInterrupts();
          sendResponse("FLIGHT MODE RESUMED.\n");
        }
      }
      else if (cmd == "STOP") {
        if (isLogging) {
          isLogging = false;
          detachSensorInterrupts();
          vTaskDelay(pdMS_TO_TICKS(100));

          if (xSemaphoreTake(flashMutex, portMAX_DELAY) == pdTRUE) {
            logger.forceFlushBuffer();
            xSemaphoreGive(flashMutex);
          }
          sendResponse("FLIGHT MODE SUSPENDED.\n");
        }
      }
      else if (cmd == "CALIBRATE") {
        sendResponse("CALIBRATION START...\n");
        performCalibration();
        sendResponse("CALIBRATION DONE.\n");
      }
      else if (cmd == "PARSE") {
        bool wasLogging = isLogging;
        if (wasLogging) {
          isLogging = false;
          detachSensorInterrupts();
          vTaskDelay(pdMS_TO_TICKS(100));
        }
        if (xSemaphoreTake(flashMutex, portMAX_DELAY) == pdTRUE) {
          sendResponse("DUMP START...\n");
          logger.forceFlushBuffer();
          logger.dumpRawBinary(Serial);
          sendResponse("DUMP DONE...\n");
          xSemaphoreGive(flashMutex);
        }
        if (wasLogging) {
          attachSensorInterrupts();
          isLogging = true;
        }
      }
      else if (cmd == "ERASE") {
        bool wasLogging = isLogging;
        if (wasLogging) {
          isLogging = false;
          detachSensorInterrupts();
          vTaskDelay(pdMS_TO_TICKS(100));
        }
        if (xSemaphoreTake(flashMutex, portMAX_DELAY) == pdTRUE) {
          sendResponse("FLASH ERASING...\n");
          logger.eraseAll();
          xSemaphoreGive(flashMutex);
        }
        sendResponse("FLASH ERASED.\n");
        if (wasLogging) {
          attachSensorInterrupts();
          isLogging = true;
        }
      }
      else if (cmd == "REBOOT") {
        sendResponse("REBOOTING...\n");
        vTaskDelay(pdMS_TO_TICKS(200));
        ESP.restart();
      }
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// ============================================================
// Core 1 : 센서 수집 태스크
// ============================================================
void IMU_Task(void *pvParameters) {
  Raw_imu raw;
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    uint32_t ts = getTimeUs32();
    if (xSemaphoreTake(spiMutex, portMAX_DELAY) == pdTRUE) {
      raw.timestamp = ts;
      imu.readRawIMU(raw.gx, raw.gy, raw.gz, raw.ax, raw.ay, raw.az);
      xSemaphoreGive(spiMutex);
    }
    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
      nav.updateIMU(raw);
      xSemaphoreGive(dataMutex);
    }
    xTaskNotify(TaskHandle_NAV, EVENT_IMU_UPDATE, eSetBits);
  }
}

void BMP_Task(void *pvParameters) {
  Raw_press raw_press;
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if (xSemaphoreTake(spiMutex, portMAX_DELAY) == pdTRUE) {
      raw_press.timestamp = getTimeUs32();
      bmp.readData(raw_press.p);
      xSemaphoreGive(spiMutex);
    }
    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
      nav.updatePress(raw_press);
      xSemaphoreGive(dataMutex);
    }
    xTaskNotify(TaskHandle_NAV, EVENT_BMP_UPDATE, eSetBits);
  }
}

// ============================================================
// Core 1 : NAV 처리 + 로그 패킷 생성 → Queue
// ============================================================
void NAV_Task(void *pvParameters) {
  uint32_t notifiedValue;
  int64_t lastImuTime_us = 0;
  for (;;) {
    xTaskNotifyWait(0x00, 0xFFFFFFFF, &notifiedValue, portMAX_DELAY);
    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
      if (notifiedValue & EVENT_IMU_UPDATE) {
        Raw_imu ri = nav.getRaw_imu();
        int64_t now_us = getTimeUs();

        // dt 계산 (64-bit 오버플로우 안전)
        float dt = (lastImuTime_us > 0)
                   ? (float)(now_us - lastImuTime_us) * 1e-6f
                   : 0.0f;
        lastImuTime_us = now_us;

        // ★ EKF predict 삽입 지점
        // nav.ekfPredict(state_imu, dt);

        if (isLogging) {
          imu_pkt pkt;
          pkt.header.SYNC_BYTE = 0xAA;
          pkt.header.id = 2;
          pkt.header.len = sizeof(imu_pkt);
          pkt.t = (uint32_t)(now_us & 0xFFFFFFFF);
          pkt.rawacc_x  = ri.ax;
          pkt.rawacc_y  = ri.ay;
          pkt.rawacc_z  = ri.az;
          pkt.rawgyro_x = ri.gx;
          pkt.rawgyro_y = ri.gy;
          pkt.rawgyro_z = ri.gz;
          LogItem item;
          item.len = sizeof(imu_pkt);
          memcpy(item.data, &pkt, item.len);
          xQueueSend(logQueue, &item, 0);
        }
      }
      if (notifiedValue & EVENT_BMP_UPDATE) {
        RocketState_PRESS ps = nav.getState_press();

        // ★ EKF update(baro) 삽입 지점
        // nav.ekfUpdateBaro(ps.altitude);

        if (isLogging) {
          baro_pkt pkt;
          pkt.header.SYNC_BYTE = 0xAA;
          pkt.header.id = 1;
          pkt.header.len = sizeof(baro_pkt);
          pkt.t = getTimeUs32();
          pkt.rawalt = ps.altitude;
          LogItem item;
          item.len = sizeof(baro_pkt);
          memcpy(item.data, &pkt, item.len);
          xQueueSend(logQueue, &item, 0);
        }
      }
      xSemaphoreGive(dataMutex);
    }
  }
}

// ============================================================
// Core 0 : Flash 기록 태스크
// ============================================================
void FlushTask(void *pvParameters) {
  LogItem item;
  for (;;) {
    // MX25Logger 내부에 _bufferMutex가 있으므로 여기서 flashMutex 불필요
    // flashMutex는 STOP/PARSE/ERASE 명령 시에만 사용
    if (xQueueReceive(logQueue, &item, pdMS_TO_TICKS(10)) == pdTRUE) {
      logger.appendRaw(item.data, item.len);
    }
    if (logger.hasFullPage()) {
      logger.flushPages();
    }
  }
}

// ============================================================
// setup
// ============================================================
void setup() {
  Serial.begin(921600);

  spiMutex = xSemaphoreCreateMutex();
  dataMutex = xSemaphoreCreateMutex();
  flashMutex = xSemaphoreCreateMutex();
  logQueue = xQueueCreate(LOG_QUEUE_LENGTH, sizeof(LogItem));
  if (!spiMutex || !dataMutex || !flashMutex || !logQueue) {
    sendResponse("동기화 객체 생성 에러\n");
    while (1);
  }

  initBLE("2026ALTIS");
  sensorSPI.begin(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN);
  // flashSPI.begin()은 logger.begin() 내부에서 호출되므로 중복 호출 불필요

  if (!imu.begin()) { sendResponse("LSM6DSO32 에러!\n"); while (1); }
  if (!bmp.begin())  { sendResponse("BMP388 에러!\n");    while (1); }
  sendResponse("모든 센서 통신 성공!");

  logger.begin(&flashSPI, FLASH_SCK_PIN, FLASH_MISO_PIN, FLASH_MOSI_PIN, FLASH_CS_PIN);

  pinMode(IMU_INT1_PIN, INPUT_PULLDOWN);
  pinMode(BMP_INT_PIN, INPUT);

  // 명령 처리 태스크 (Core 0)
  xTaskCreatePinnedToCore(processCommandTask, "CmdTask", 4096, NULL, 1, NULL, 0);

  sendResponse(">>> WAITING FOR START COMMAND...\n");

  // START 명령 대기
  while (!systemStarted) {
    vTaskDelay(pdMS_TO_TICKS(10));
  }

  // 센서 버퍼 클리어
  int16_t d1, d2, d3, d4, d5, d6; float d0;
  imu.readRawIMU(d1, d2, d3, d4, d5, d6);
  bmp.readData(d0);

  // 태스크 생성
  isLogging = true;
  xTaskCreatePinnedToCore(IMU_Task, "IMU_T", 4096, NULL, 5, &TaskHandle_IMU, 1);
  xTaskCreatePinnedToCore(BMP_Task, "BMP_T", 4096, NULL, 4, &TaskHandle_BMP, 1);
  xTaskCreatePinnedToCore(NAV_Task, "NAV_T", 8192, NULL, 3, &TaskHandle_NAV, 1);
  xTaskCreatePinnedToCore(FlushTask, "Flush_T", 4096, NULL, 2, &FlushTaskHandle, 0);

  // 인터럽트 연결
  attachSensorInterrupts();
  imu.enableAccelDataReadyInterrupt(1);

  sendResponse("FLIGHT SYSTEM ACTIVE\n");
}

void loop() {}
