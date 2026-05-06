#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <esp_timer.h>

#include "Config.h"
#include "LSM6DSO32.h"
#include "BMP388.h"
#include "MMC5983MA.h"
#include "NEOM9N.h"
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
static MMC5983MA mag(&Wire);
static NEOM9N    gps(Serial1, GPS_RX_PIN, GPS_TX_PIN);
static NAV       nav;
static MX25Logger logger;

static TaskHandle_t TaskHandle_IMU   = NULL;
static TaskHandle_t TaskHandle_BMP   = NULL;
static TaskHandle_t TaskHandle_MAG   = NULL;
static TaskHandle_t TaskHandle_GPS   = NULL;
static TaskHandle_t FlightTaskHandle = NULL;
static TaskHandle_t FlushTaskHandle  = NULL;

static SemaphoreHandle_t spiMutex   = NULL;
static SemaphoreHandle_t i2cMutex   = NULL;
static SemaphoreHandle_t navMutex   = NULL;
static SemaphoreHandle_t flashMutex = NULL;

static volatile bool flightActive = false;
static FlightPhase flightPhase = FlightPhase::PRE_FLIGHT;

// Forward Declarations
void IMU_Task(void *pvParameters);
void BMP_Task(void *pvParameters);
void MAG_Task(void *pvParameters);
void GPS_Task(void *pvParameters);
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
  i2cMutex   = xSemaphoreCreateMutex();
  navMutex   = xSemaphoreCreateMutex();
  flashMutex = xSemaphoreCreateMutex();
  if (!spiMutex || !i2cMutex || !navMutex || !flashMutex) {
    beep(100, 3);
    while (1);
  }

  // 3. Communications
  initBLE(BLE_DEVICE_NAME);
  sensorSPI.begin(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN);
  Wire.begin(MAG_SDA_PIN, MAG_SCL_PIN, 400000);

  // 4. Sensors
  bool ok = true;
  if (!imu.begin()) { sendResponse("IMU FAIL\n"); ok = false; }
  if (!bmp.begin()) { sendResponse("BMP FAIL\n"); ok = false; }
  if (!mag.begin()) { sendResponse("MAG FAIL\n"); ok = false; }
  if (!gps.begin(10)) { sendResponse("GPS WARN\n"); } // Non-fatal
  
  if (!ok) { beep(100, 3); while(1); }
  sendResponse("ALL SENSORS OK\n");

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
  xTaskCreatePinnedToCore(MAG_Task, "MAG_T",  STACK_SIZE_SENSOR, NULL, TASK_C0_PRIO_MAG, &TaskHandle_MAG, 0);
  xTaskCreatePinnedToCore(GPS_Task, "GPS_T",  STACK_SIZE_SENSOR, NULL, TASK_C0_PRIO_GPS, &TaskHandle_GPS, 0);
  xTaskCreatePinnedToCore(Flight_Task,"Flt_T", 4096,              NULL, 3,                 &FlightTaskHandle, 0);
  xTaskCreatePinnedToCore(FlushTask,"Flush_T",STACK_SIZE_FLUSH,  NULL, TASK_C0_PRIO_FLUSH, &FlushTaskHandle, 0);

  beep(200);
  sendResponse(">>> READY\n");
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
    else if (cmd == "CALIBRATE_MAG") {
      digitalWrite(LED_PIN, HIGH);
      bool magOk = false;
      while (!magOk) {
        sendResponse("CALIBRATING MAG (30S)...\n");
        if (mag.calibrate(MAG_CALIB_MS)) {
          mag.clearInterruptFlag();
          magOk = true;
          sendResponse("MAG CALIB DONE.\n");
          digitalWrite(LED_PIN, LOW); beep(200);
        } else {
          sendResponse("INSUFFICIENT ROTATION - RETRYING...\n");
          beep(100, 3);
          vTaskDelay(pdMS_TO_TICKS(2000));
        }
      }
    }
    else if (cmd == "CALIBRATE_GPS") {
      sendResponse("CALIBRATING GPS (AVERAGING)...\n");
      bool gpsOk = false;
      while (!gpsOk) {
        if (gps.calibrate()) {
          gpsOk = true;
          sendResponse("GPS ORIGIN OK\n");
          beep(200);
        } else {
          sendResponse("GPS DRIFTING OR NO FIX - RETRYING...\n");
          beep(100, 3);
          vTaskDelay(pdMS_TO_TICKS(2000));
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
      sendResponse("ALIGNING EKF...\n");
      digitalWrite(LED_PIN, HIGH);

      // 수렴 판정 파라미터
      const int   MIN_ITER     = 10;     // 최소 1.0s (조기 false-positive 방지)
      const int   MAX_ITER     = 200;    // 최대 20.0s (안전 상한)
      const float REL_THRESH   = 1e-3f;  // P 상대 변화 0.1% 이하면 안정
      const int   STABLE_REQ   = 3;      // 연속 안정 횟수

      float P_prev = 0.0f;
      int   stable_count = 0;
      int   iter = 0;
      bool  converged = false;

      for (iter = 0; iter < MAX_ITER; iter++) {
        if (xSemaphoreTake(navMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
          if (!nav.isEkfReady()) nav.ekfBegin();
          nav.ekfUpdateStaticAlignment();
          float P_now = nav.ekf().attBiasCovTrace();
          xSemaphoreGive(navMutex);

          if (iter >= MIN_ITER && P_prev > 0.0f) {
            float rel = fabsf(P_prev - P_now) / P_prev;
            if (rel < REL_THRESH) {
              if (++stable_count >= STABLE_REQ) { converged = true; break; }
            } else {
              stable_count = 0;
            }
          }
          P_prev = P_now;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
      }

      char buf[80];
      snprintf(buf, sizeof(buf), "ZUPT %s in %.1fs (P_trace=%.3e)\n",
               converged ? "CONVERGED" : "TIMEOUT",
               (iter + 1) * 0.1f, P_prev);
      sendResponse(buf);
      digitalWrite(LED_PIN, LOW); beep(100);
    }
    else if (cmd == "START") {
      sendResponse("STARTING...\n");
      if (xSemaphoreTake(navMutex, portMAX_DELAY) == pdTRUE) {
        if (!nav.isEkfReady()) {
          if (!nav.ekfBegin()) {
            sendResponse("EKF INIT FAIL\n");
            xSemaphoreGive(navMutex);
            beep(100, 3); return;
          }
        }
        xSemaphoreGive(navMutex);
      }
      sendResponse("EKF READY\n");

      clearSensors();
      mag.clearInterruptFlag();
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
    // START 명령이 올 때까지 무한 대기 (CPU 점유 0)
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    
    flightPhase = FlightPhase::PRE_FLIGHT;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (flightActive) {
      vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100)); // 10Hz

      // Non-blocking pyro control: Check if 1000ms has passed since activation
      if (pyro1_active && (millis() - pyro1_start_ms > 1000)) {
        digitalWrite(PYRO_1_PIN, LOW);
        pyro1_active = false;
      }

      State_nominal s;
      State_imu imu_state;
      if (xSemaphoreTake(navMutex, 0) == pdTRUE) {
        s = nav.getNominal();
        imu_state = nav.getStateImu();
        xSemaphoreGive(navMutex);
      } else continue;

      float alt = -s.p[2];
      float vel_up = -s.v[2];
      
      // 현재 가속도 크기 계산 (EKF가 추정한 동적 바이어스까지 제거)
      float ax = imu_state.ax - s.ba[0];
      float ay = imu_state.ay - s.ba[1];
      float az = imu_state.az - s.ba[2];
      float amag = sqrtf(ax*ax + ay*ay + az*az);

      switch (flightPhase) {
        case FlightPhase::PRE_FLIGHT:
          if (alt > 5.0f || vel_up > 5.0f) {
            flightPhase = FlightPhase::POWERED_FLIGHT;
            logger.logEvent(flightPhase, 1);
            sendResponse("LAUNCH\n");
          }
          break;

        case FlightPhase::POWERED_FLIGHT:
          static bool reached_high_g = false;
          if (amag > 20.0f) reached_high_g = true;
          
          if (reached_high_g && amag < 10.0f) {
            flightPhase = FlightPhase::COASTING;
            logger.logEvent(flightPhase, 2);
            sendResponse("BO\n");
            reached_high_g = false;
          }
          break;

        case FlightPhase::COASTING: {
          // Apogee detection: 3 consecutive samples (300ms) with descent speed
          static uint8_t descent_count = 0;
          if (vel_up < -1.0f && alt > 15.0f) { // Threshold tightened: -1.0m/s, >15m alt
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

void MAG_Task(void *pvParameters) {
  Raw_mag m;
  for (;;) {
    bool got = false;
    if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE) {
      if (mag.isDataReady()) {
        m.timestamp = (uint32_t)(esp_timer_get_time() & 0xFFFFFFFF);
        mag.readCalibratedMag(m.mx, m.my, m.mz);
        mag.clearInterruptFlag();
        got = true;
      }
      xSemaphoreGive(i2cMutex);
    }
    if (!got) { vTaskDelay(pdMS_TO_TICKS(MAG_POLL_MS)); continue; }
    
    if (flightActive) logger.logMag(m);
    
    if (xSemaphoreTake(navMutex, portMAX_DELAY) == pdTRUE) {
      nav.updateMag(m); 
      xSemaphoreGive(navMutex);
    }
  }
}

void GPS_Task(void *pvParameters) {
  Raw_gps g;
  for (;;) {
    if (gps.update()) {
      g.timestamp = (uint32_t)(esp_timer_get_time() & 0xFFFFFFFF);
      g.hasPos    = gps.getNED(g.pn, g.pe, g.pd, g.vn, g.ve, g.vd, g.hAcc, g.vAcc, g.fixType, g.numSV);
      
      if (flightActive) logger.logGps(g);
      
      if (xSemaphoreTake(navMutex, portMAX_DELAY) == pdTRUE) {
        nav.updateGps(g); 
        xSemaphoreGive(navMutex);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(GPS_POLL_MS));
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
