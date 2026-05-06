#ifndef PYTHON_HEADER_H
#define SENSOR_DATA_H

#include <stdint.h>

#pragma pack(push, 1)

// ============================================================
// Packet IDs (Used by Python Parser)
// ============================================================
#define ID_BARO  1
#define ID_IMU   2
#define ID_MAG   3
#define ID_GPS   4
#define ID_STATE 5

struct PacketHeader {
  uint8_t SYNC_BYTE = 0xAA;
  uint8_t id;
  uint8_t len;
};
// ID 1 : Barometer altitude
struct baro_pkt {
  PacketHeader header;
  uint32_t t;
  float alt;
};

// ID 2 : IMU raw
struct imu_pkt {
  PacketHeader header;
  uint32_t t;
  int16_t gx;
  int16_t gy;
  int16_t gz;
  int16_t ax;
  int16_t ay;
  int16_t az;
};

// ID 3 : Magnetometer
struct mag_pkt {
  PacketHeader header;
  uint32_t t;
  float mx;
  float my;
  float mz;
};

// ID 4 : GPS NED
struct gps_pkt {
  PacketHeader header;
  uint32_t t;
  float pn;
  float pe;
  float pd;
  float vn;
  float ve;
  float vd;
  float hAcc;
  float vAcc;
  uint8_t fixType;
  uint8_t numSV;
};

// ID 5 : ES-EKF nominal state
struct state_pkt {
  PacketHeader header;
  uint32_t t;
  float pn;
  float pe;
  float pd;
  float vn;
  float ve;
  float vd;
  float qw;
  float qx;
  float qy;
  float qz;
  float ba_x;
  float ba_y;
  float ba_z;
  float bg_x;
  float bg_y;
  float bg_z;
};

#pragma pack(pop)

#endif
