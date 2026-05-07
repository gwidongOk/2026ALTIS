#ifndef SENSOR_DATA_H
#define SENSOR_DATA_H

#include <stdint.h>

// ============================================================
// Packet IDs (Used by MX25Logger and Python Parser)
// ============================================================
#define ID_BARO  1
#define ID_IMU   2
#define ID_MAG   3
#define ID_GPS   4
#define ID_STATE 5
#define ID_EVENT 6

// ============================================================
// In-RAM sensor data (not packed — used in tasks, NAV, queues)
// ============================================================

// IMU — axis-aligned + int16 bias removed (by LSM6DSO32::readCalibratedIMU)
struct Raw_imu {
    uint32_t timestamp;
    int16_t gx, gy, gz;
    int16_t ax, ay, az;
};

// BARO — altitude (m) above pad (from BMP388::readAltitude)
struct Raw_press {
    uint32_t timestamp;
    float alt;
};

// MAG — fully calibrated Gauss (from MMC5983MA::readCalibratedMag)
struct Raw_mag {
    uint32_t timestamp;
    float mx, my, mz;
};

// GPS — NED pos/vel + accuracy + fix (from NEOM9N::getNED)
struct Raw_gps {
    uint32_t timestamp;
    float pn, pe, pd;
    float vn, ve, vd;
    float hAcc, vAcc;
    uint8_t fixType;
    uint8_t numSV;
    bool hasPos;   // true if origin set AND fix>=3
};

// SI-converted IMU state (float), used by EKF predict
struct State_imu {
    uint32_t timestamp;
    float ax, ay, az;   // [m/s^2]
    float gx, gy, gz;   // [rad/s]
};

// ES-EKF nominal state (16 floats)
struct State_nominal {
    uint32_t timestamp;
    float p[3];   // NED position [m]
    float v[3];   // NED velocity [m/s]
    float q[4];   // quaternion (w, x, y, z)
    float ba[3];  // accel bias
    float bg[3];  // gyro bias
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

// ID 2 : IMU calibrated raw (axis-aligned + int16 bias subtracted)
struct imu_pkt {
  PacketHeader header;
  uint32_t t;
  int16_t gx, gy, gz;
  int16_t ax, ay, az;
};

// ID 3 : Magnetometer fully calibrated Gauss (hard+soft iron + axis)
struct mag_pkt {
  PacketHeader header;
  uint32_t t;
  float mx, my, mz;
};

// ID 4 : GPS NED from origin + velocity + accuracy + fix info
struct gps_pkt {
  PacketHeader header;
  uint32_t t;
  float pn, pe, pd;
  float vn, ve, vd;
  float hAcc, vAcc;
  uint8_t fixType;
  uint8_t numSV;
};

// ID 5 : ES-EKF nominal state
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
