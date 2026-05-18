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

// Magnetometer I2C
#define MAG_SDA_PIN  5
#define MAG_SCL_PIN  4

// GPS UART (Serial1) — NOTE: GPS_TX_PIN (1) is repurposed for STAGE_IR_PIN below
#define GPS_RX_PIN   2
//#define GPS_TX_PIN 1   // disabled: shared with STAGE_IR_PIN

// Stage-separation IR sensor (FC-51 digital out): HIGH = separated, LOW = still attached
#define STAGE_IR_PIN  1

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
// RTOS Task Settings (Priority: 5 is highest)
// ============================================================

// Core 1: High-speed Real-time Sensors (Dedicated)
#define TASK_C1_PRIO_IMU     5
#define TASK_C1_PRIO_BMP     4

// Core 0: System tasks
#define TASK_C0_PRIO_FLUSH   2  // Logging is lower than sensor reading

#define STACK_SIZE_SENSOR    8192
#define STACK_SIZE_FLUSH     4096

// ============================================================
// Sensor & Logic Parameters
// ============================================================
#define CALIB_SAMPLES     100

// ============================================================
// Flight Profile (2-stage)
// ============================================================
#define STAGE2_PULSE_MS     1000    // PYRO_1 (re-ignition) pulse width
#define MAIN_DEPLOY_ALT_M   100.0f  // Altitude (AGL) to trigger PYRO_2 (main)
#define MAIN_PULSE_MS       1000    // PYRO_2 pulse width
#define TILT_LIMIT_DEG      45.0f   // Abort 2nd-stage ignition if tilt from vertical exceeds

// SERVO_1: drogue parachute release at apogee
#define SERVO_DROGUE_IDLE_DEG    92     // Pre-flight / armed (latched)
#define SERVO_DROGUE_DEPLOY_DEG  0    // Apogee deploy angle

#endif
