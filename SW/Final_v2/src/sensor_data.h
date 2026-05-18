#ifndef SENSOR_DATA_H
#define SENSOR_DATA_H

#include <stdint.h>

// ============================================================
// Packet IDs (Used by MX25Logger and Python Parser)
// ============================================================
#define ID_BARO  1
#define ID_IMU   2
#define ID_STATE 5
#define ID_EVENT 6

// ============================================================
// In-RAM sensor data (not packed — used in tasks, NAV, queues)
// ============================================================

// IMU — axis-aligned + int16 bias removed
struct Raw_imu {
    uint32_t timestamp;
    int16_t gx, gy, gz;
    int16_t ax, ay, az;
};

// BARO — altitude (m) above pad
struct Raw_press {
    uint32_t timestamp;
    float alt;
};

// SI-converted IMU state (float), used by EKF predict
struct State_imu {
    uint32_t timestamp;
    float ax, ay, az;   // [m/s^2]
    float gx, gy, gz;   // [rad/s]
};

// Navigation nominal state (NED frame)
//   p[3] = NED position  [North, East, Down]   — NE: dead-reckoned, D: 1D KF
//   v[3] = NED velocity  [North, East, Down]   — NE: dead-reckoned, D: 1D KF
//   q[4] = orientation quaternion  [w, x, y, z] — gyro integration
struct State_nominal {
    uint32_t timestamp;
    float p[3];
    float v[3];
    float q[4];
};

// ============================================================
// On-flash packets (packed — written to MX25 flash via MX25Logger)
// ============================================================
#pragma pack(push, 1)

struct PacketHeader {
  uint8_t SYNC_BYTE = 0xAA;
  uint8_t id;
  uint8_t len;
};

// ID 1 : Barometer altitude (m), pad-referenced
struct baro_pkt {
  PacketHeader header;
  uint32_t t;
  float alt;
};

// ID 2 : IMU calibrated raw
struct imu_pkt {
  PacketHeader header;
  uint32_t t;
  int16_t gx, gy, gz;
  int16_t ax, ay, az;
};

// ID 5 : Navigation state (NED p/v + quaternion)
struct state_pkt {
  PacketHeader header;
  uint32_t t;
  float p[3];
  float v[3];
  float q[4];
};

enum FlightPhase : uint8_t {
  PRE_FLIGHT = 0,
  POWERED_FLIGHT,
  COASTING,
  DESCENT,
  LANDED
};

// ID 6 : Flight Event / State change
struct event_pkt {
  PacketHeader header;
  uint32_t t;
  uint8_t  phase;    // Current FlightPhase
  uint8_t  event_id; // 1:Launch, 2:Burnout, 3:Apogee, 4:Landing, 5:NotStageCondition
};

#pragma pack(pop)

#endif
