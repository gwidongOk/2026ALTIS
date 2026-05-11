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

// ES-EKF nominal state (simplified for 1D, but fields kept for logging consistency)
struct State_nominal {
    uint32_t timestamp;
    float p[3];   // [0, 0, altitude]
    float v[3];   // [0, 0, vertical_vel]
    float q[4];   // [1, 0, 0, 0]
    float ba[3];  // [0, 0, accel_bias]
    float bg[3];  // [0, 0, 0]
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

// ID 5 : State (Simplified 1D)
struct state_pkt {
  PacketHeader header;
  uint32_t t;
  float p[3];
  float v[3];
  float q[4];
  float ba[3];
  float bg[3];
};

enum struct FlightPhase : uint8_t {
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
  uint8_t  event_id; // 0:None, 1:Launch, 2:Apogee, 3:Landing
};

#pragma pack(pop)

#endif
