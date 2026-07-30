#ifndef CONFIG_H
#define CONFIG_H

// ============================================================
// Hardware pins (2026 ALTIS AVIONICS V1.1)
// ============================================================

// Sensor SPI (IMU + BARO)
#define SPI_SCK_PIN   13
#define SPI_MISO_PIN  12
#define SPI_MOSI_PIN  11
#define IMU_CS_PIN    14
#define BMP_CS_PIN    10
#define IMU_INT1_PIN  47
#define BMP_INT_PIN   9

// Flash SPI
#define FLASH_SCK_PIN   16
#define FLASH_MISO_PIN  7
#define FLASH_MOSI_PIN  15
#define FLASH_CS_PIN    6

// Stage-separation Hall sensor, external 4.7k pull-up already on the board.
// Bench-confirmed flight logic:
//   LOW  = stages joined
//   HIGH = stage separation confirmed
#define STAGE_SEP_PIN  1

//BUZZER
#define BUZZER_PIN   17

//SERVO
#define SERVO_1_PIN   41
#define SERVO_2_PIN   40
#define SERVO_3_PIN   39
#define SERVO_4_PIN   38

//Pyro
#define PYRO_1_PIN   18
#define PYRO_2_PIN   8

// ============================================================
// Communication Settings
// ============================================================
#define SERIAL_BAUD       921600
#define BLE_DEVICE_NAME   "2026ALTIS"

// ============================================================
// RTOS Task Settings
// ============================================================

// Core 1 owns every real-time object: both sensors, NAV, flight state and
// actuators. Core 0 owns command reception and flash draining.
#define TASK_C1_PRIO_REALTIME  5
#define TASK_C0_PRIO_STORAGE   2

// CALIBRATE and READY alignment execute inside the real-time task before
// arming, so leave generous stack headroom for their local state/Eigen calls.
#define STACK_SIZE_REALTIME    16384
#define STACK_SIZE_STORAGE      6144
#define COMMAND_QUEUE_LENGTH      16

// ============================================================
// Sensor & Logic Parameters
// ============================================================
// Vertical IMU calibration:
// 5 s warm-up once, then robust 512-sample windows until three consecutive
// stable windows pass. Abort after 60 s instead of retrying forever.
#define IMU_CALIB_WARMUP_MS       5000
#define IMU_CALIB_WINDOW_SAMPLES  512
#define IMU_CALIB_STABLE_WINDOWS  3
#define IMU_CALIB_TIMEOUT_MS      60000
#define BARO_ZERO_SAMPLES         100

// READY is expected to last about one minute.
#define READY_GYRO_BIAS_TAU_S     10.0f
#define READY_BARO_REF_TAU_S      15.0f
#define READY_MIN_VALID_S         30.0f
#define READY_STABLE_MIN_S        5.0f

// Initial stationarity gates; tune from pad test data if necessary.
#define READY_ACCEL_NORM_TOL_MPS2 1.0f
#define READY_GYRO_RATE_LIMIT_DPS 3.0f

// Launch detection / verification:
// one axial 3 g sample starts a 1 s candidate window; barometric altitude
// must then exceed 10 m before any buffered data is committed to flash.
#define LAUNCH_ACCEL_THRESHOLD_G  3.0f
#define LAUNCH_VERIFY_MS          1000
#define LAUNCH_VERIFY_ALT_M       10.0f

// PSRAM-only ring buffer while READY/ARMED/VERIFY. Old records are overwritten
// at capacity; the buffer reaches flash only after launch confirmation.
#define PRETRIGGER_RECORD_CAPACITY 1600

// ============================================================
// Flight Profile (2-stage)
// ============================================================
#define STAGE2_PULSE_MS     1000    // PYRO_1 (re-ignition) pulse width
#define STAGE2_IGNITION_TIMEOUT_MS 2000 // No stage-2 ignition after COASTING -> apogee/drogue
#define STAGE2_APOGEE_TIMEOUT_MS 8500 // Backup apogee after stage-2 ignition
#define BURNOUT_TIMEOUT_MS  2000    // Fallback: force burnout transition 2 s after launch detection
#define MAIN_DEPLOY_ALT_M   100.0f  // Altitude (AGL) to trigger PYRO_2 (main)
#define MAIN_PULSE_MS       1000    // PYRO_2 pulse width
#define TILT_LIMIT_DEG      45.0f   // Abort 2nd-stage ignition if tilt from vertical exceeds

// SERVO_1: drogue parachute release at apogee
#define SERVO_DROGUE_IDLE_DEG    90     // Pre-flight / armed (latched)
#define SERVO_DROGUE_DEPLOY_DEG  0    // Apogee deploy angle

#endif
