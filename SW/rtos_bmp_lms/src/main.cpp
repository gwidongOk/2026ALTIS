#include <Arduino.h>
#include <SPI.h>
#include "LSM6DSO32.h"
#include "BMP388.h"
#include "NAV.h"

const uint8_t SPI_SCK_PIN = 13;
const uint8_t SPI_MISO_PIN = 12;
const uint8_t SPI_MOSI_PIN = 11;
const uint8_t IMU_CS_PIN = 14;
const uint8_t BMP_CS_PIN = 10;
const uint8_t IMU_INT1_PIN = 47;
const uint8_t IMU_INT2_PIN = 21;
const uint8_t BMP_INT_PIN = 9;

// ========================================================
// ★ 어떤 센서가 업데이트 되었는지 구분하기 위한 비트마스크
// ========================================================
#define EVENT_ACC_UPDATE   (1 << 0) // 0b001
#define EVENT_GYRO_UPDATE  (1 << 1) // 0b010
#define EVENT_BMP_UPDATE   (1 << 2) // 0b100

SPIClass sensorSPI(HSPI);
LSM6DSO32 imu(IMU_CS_PIN, &sensorSPI);
BMP388 bmp(BMP_CS_PIN, &sensorSPI);
NAV nav;

TaskHandle_t TaskHandle_ACC = NULL;
TaskHandle_t TaskHandle_GYRO = NULL;
TaskHandle_t TaskHandle_BMP = NULL;
TaskHandle_t TaskHandle_NAV = NULL;

SemaphoreHandle_t spiMutex;
SemaphoreHandle_t dataMutex;

void IRAM_ATTR ACCInterruptHandler()
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  vTaskNotifyGiveFromISR(TaskHandle_ACC, &xHigherPriorityTaskWoken);
  if (xHigherPriorityTaskWoken)
    portYIELD_FROM_ISR();
}

void IRAM_ATTR GYROInterruptHandler()
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  vTaskNotifyGiveFromISR(TaskHandle_GYRO, &xHigherPriorityTaskWoken);
  if (xHigherPriorityTaskWoken)
    portYIELD_FROM_ISR();
}

void IRAM_ATTR BMPInterruptHandler()
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  vTaskNotifyGiveFromISR(TaskHandle_BMP, &xHigherPriorityTaskWoken);
  if (xHigherPriorityTaskWoken)
    portYIELD_FROM_ISR();
}

void ACC_Task(void *pvParameters)
{
  Raw_imu raw_acc;
  for (;;)
  {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if (xSemaphoreTake(spiMutex, portMAX_DELAY) == pdTRUE)
    {
      raw_acc.timestamp = millis();
      imu.readRawAccel(raw_acc.x, raw_acc.y, raw_acc.z);
      xSemaphoreGive(spiMutex);
    }
    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE)
    {
      nav.updateAccel(raw_acc);
      xSemaphoreGive(dataMutex);
    }
    // ★ 가속도 업데이트 완료 비트 전송
    xTaskNotify(TaskHandle_NAV, EVENT_ACC_UPDATE, eSetBits);
  }
}

void GYRO_Task(void *pvParameters)
{
  Raw_imu raw_gyro;
  for (;;)
  {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if (xSemaphoreTake(spiMutex, portMAX_DELAY) == pdTRUE)
    {
      raw_gyro.timestamp = millis();
      imu.readRawGyro(raw_gyro.x, raw_gyro.y, raw_gyro.z);
      xSemaphoreGive(spiMutex);
    }
    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE)
    {
      nav.updateGyro(raw_gyro);
      xSemaphoreGive(dataMutex);
    }
    // ★ 자이로 업데이트 완료 비트 전송
    xTaskNotify(TaskHandle_NAV, EVENT_GYRO_UPDATE, eSetBits);
  }
}

void BMP_Task(void *pvParameters)
{
  Raw_press raw_press;
  for (;;)
  {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if (xSemaphoreTake(spiMutex, portMAX_DELAY) == pdTRUE)
    {
      raw_press.timestamp = millis();
      bmp.readData(raw_press.p);
      xSemaphoreGive(spiMutex);
    }
    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE)
    {
      nav.updatePress(raw_press);
      xSemaphoreGive(dataMutex);
    }
    // ★ 기압계 업데이트 완료 비트 전송
    xTaskNotify(TaskHandle_NAV, EVENT_BMP_UPDATE, eSetBits);
  }
}

void NAV_Task(void *pvParameters)
{
  Raw_imu raw_acc;
  Raw_imu raw_gyro;
  RocketState_imu acc_state;
  RocketState_imu gyro_state;
  RocketState_PRESS press_state;
  
  uint32_t notifiedValue; // 어떤 센서가 깨웠는지 알림 값을 받을 변수

  for (;;)
  {
    xTaskNotifyWait(0x00, 0xFFFFFFFF, &notifiedValue, portMAX_DELAY);

    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE)
    {
      // 1. 가속도 데이터가 업데이트 되었을 때만 실행
      if (notifiedValue & EVENT_ACC_UPDATE) {
          raw_acc = nav.getraw_acc();
          acc_state = nav.getState_acc();
        static int printCount = 0;
        static unsigned long timestamp_prev = 0;
        printCount++;
        if (millis() - timestamp_prev >= 1000)
        { float hz = printCount;
          timestamp_prev = millis();
          printf("acc %0.2f Hz , %0.2f m/s^2\n",
                 hz, acc_state.x);
          printCount = 0;
        }
      }

      // 2. 자이로 데이터가 업데이트 되었을 때만 실행
      if (notifiedValue & EVENT_GYRO_UPDATE) {
          raw_gyro = nav.getraw_gyro();
          gyro_state = nav.getState_gyro();
        static int printCount = 0;
        static unsigned long timestamp_prev = 0;
        printCount++;
        if (millis() - timestamp_prev >= 1000)
        { float hz = printCount;
          timestamp_prev = millis();

          printf("gyro %0.2f Hz , %0.2f deg/s\n",
                 hz, gyro_state.x);
          printCount = 0;
        }
      }

      // 3. 기압계 데이터가 업데이트 되었을 때만 실행
      if (notifiedValue & EVENT_BMP_UPDATE)
      {
        press_state = nav.getState_press();
        // (디버그용: 확인을 위해 간헐적으로 출력, 실제 로깅 시에는 제거 권장)
        static int printCount = 0;
        static unsigned long timestamp_prev = 0;
        printCount++;
        if (millis() - timestamp_prev >= 1000)
        { float hz = printCount;
          timestamp_prev = millis();
          bmp.readData(press_state.pressure); // 최신 기압값으로 갱신

          printf("bmp %0.2f Hz , %0.2f hpa\n",
                 hz, press_state.pressure);
          printCount = 0;
        }
      }

      xSemaphoreGive(dataMutex);
    }
  }
}

void setup()
{
  Serial.begin(115200);
  spiMutex = xSemaphoreCreateMutex();
  dataMutex = xSemaphoreCreateMutex();
  if (spiMutex == NULL || dataMutex == NULL)
  {
    Serial.println("Mutex 에러!");
    while (1)
      ;
  }

  sensorSPI.begin(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN);

  if (!imu.begin())
  {
    Serial.println("LSM6DSO32 에러!");
    while (1)
      ;
  }
  if (!bmp.begin())
  {
    Serial.println("BMP388 에러!");
    while (1)
      ;
  }
  Serial.println("모든 센서 통신 성공!");

  pinMode(IMU_INT1_PIN, INPUT_PULLDOWN);
  pinMode(IMU_INT2_PIN, INPUT_PULLDOWN);
  pinMode(BMP_INT_PIN, INPUT);

  //센서 캘리브레이션
  Serial.println("센서 워밍업 및 캘리브레이션 시작...");

  // 1초간 불안정한 데이터 버리기 (워밍업)
  for (int i = 0; i < 50; i++)
  {
    float dummy_p;
    bmp.readData(dummy_p);
    delay(20);
  }

  // 2초간 100개의 데이터 누적
  float sum_gx = 0, sum_gy = 0, sum_gz = 0;
  float sum_p = 0;
  int calib_samples = 100;

  for (int i = 0; i < calib_samples; i++)
  {
    int16_t rx, ry, rz;
    float p;

    imu.readRawGyro(rx, ry, rz);
    bmp.readData(p);

    sum_gx += rx;
    sum_gy += ry;
    sum_gz += rz;
    sum_p += p;
    delay(20);
  }

  // NAV 객체에 전달하여 초기 영점 세팅 완료
  nav.calibrateGyro(sum_gx, sum_gy, sum_gz, calib_samples);
  nav.calibratePress(sum_p, calib_samples);
  Serial.println("캘리브레이션 완료!");

  // 초기 버퍼 클리어
  float d0;
  int16_t d1, d2, d3;
  imu.readRawAccel(d1, d2, d3);
  imu.readRawGyro(d1, d2, d3);
  bmp.readData(d0);

  // Task 생성
  xTaskCreatePinnedToCore(ACC_Task, "ACC_Task", 4096, NULL, 5, &TaskHandle_ACC, 1);
  xTaskCreatePinnedToCore(GYRO_Task, "GYRO_Task", 4096, NULL, 5, &TaskHandle_GYRO, 1);
  xTaskCreatePinnedToCore(BMP_Task, "BMP_Task", 4096, NULL, 4, &TaskHandle_BMP, 1);
  xTaskCreatePinnedToCore(NAV_Task, "NAV_Task", 4096, NULL, 3, &TaskHandle_NAV, 1);

  // 하드웨어 인터럽트 연결 및 활성화
  attachInterrupt(digitalPinToInterrupt(IMU_INT1_PIN), ACCInterruptHandler, RISING);
  attachInterrupt(digitalPinToInterrupt(IMU_INT2_PIN), GYROInterruptHandler, RISING);
  attachInterrupt(digitalPinToInterrupt(BMP_INT_PIN), BMPInterruptHandler, RISING);

  imu.enableAccelDataReadyInterrupt(1);
  imu.enableGyroDataReadyInterrupt(2);

  Serial.println("RTOS 스케줄러 세팅 완료. 인터럽트 개시!");
}

void loop()
{
  vTaskDelete(NULL);
}