#include <Arduino.h>
#include <SPI.h>
#include "MX25Logger.h"
#include "sensor_data.h"


#define FLASH_SCK_PIN   16
#define FLASH_MISO_PIN  7
#define FLASH_MOSI_PIN  15
#define FLASH_CS_PIN    6

// SPI 버스(FSPI)를 독립적으로 할당합니다.
SPIClass flashSPI(FSPI);

MX25Logger logger;

bool isLogging = true;
TaskHandle_t DataGenTaskHandle;
TaskHandle_t FlushTaskHandle;

// ========================================================
// 태스크 1: 센서 데이터 생성 → 버퍼에 추가만
// ========================================================
void DataGenTask(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(50); // 50ms (20Hz)

  while (true) {
    if (isLogging) {
      baro myBaro;
      myBaro.header.id = 1;
      myBaro.header.len = sizeof(baro);
      myBaro.t = millis();
      myBaro.rawalt = 105.3f;

      imu myImu;
      myImu.header.id = 2;
      myImu.header.len = sizeof(imu);
      myImu.t = millis();
      myImu.rawacc_x = 200;
      myImu.rawacc_y = 0;
      myImu.rawacc_z = 0;
      myImu.rawgyro_x = 0;
      myImu.rawgyro_y = 0;
      myImu.rawgyro_z = 0;

      // 버퍼에 추가만 (플래시 쓰기 없음)
      logger.appendData(myBaro);
      logger.appendData(myImu);
    }
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// ========================================================
// 태스크 2: 버퍼에 256바이트 이상이면 플래시에 자동 기록
// ========================================================
void FlushTask(void *pvParameters) {
  const TickType_t xFrequency = pdMS_TO_TICKS(10); // 10ms 주기로 확인

  while (true) {
    if (isLogging && logger.hasFullPage()) {
      logger.flushPages();
    }
    vTaskDelay(xFrequency);
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) vTaskDelay(pdMS_TO_TICKS(10));
  vTaskDelay(pdMS_TO_TICKS(1000));

  logger.begin(&flashSPI, FLASH_SCK_PIN, FLASH_MISO_PIN, FLASH_MOSI_PIN, FLASH_CS_PIN);
  Serial.print("eraseAll");
  logger.eraseAll();
  Serial.print("start");
  // 데이터 생성 태스크 (코어 1)
  xTaskCreatePinnedToCore(
    DataGenTask, "DataGenTask", 4096, NULL, 1, &DataGenTaskHandle, 1
  );

  // 플래시 기록 태스크 (코어 0)
  xTaskCreatePinnedToCore(
    FlushTask, "FlushTask", 4096, NULL, 2, &FlushTaskHandle, 0
  );
}

void loop() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    if ((cmd == 'p' || cmd == 'P') && isLogging) {
      isLogging = false;
      logger.forceFlushBuffer();
      logger.dumpRawBinary(Serial);
    }
  }

  vTaskDelay(pdMS_TO_TICKS(10));
}
