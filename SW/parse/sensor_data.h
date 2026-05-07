#ifndef PYTHON_HEADER_H
#define PYTHON_HEADER_H

#include <stdint.h>

#pragma pack(push, 1)

// ============================================================
// Packet IDs
// ============================================================
#define ID_BARO  1
#define ID_IMU   2

struct PacketHeader {
  uint8_t SYNC_BYTE;
  uint8_t id;
  uint8_t len;
};

// ID 1 : Barometer altitude (m), pad-referenced
struct baro_pkt {
  PacketHeader header;
  uint32_t t;
  float alt;
};

// ID 2 : IMU calibrated (body frame, bias subtracted)
// Scale: gyro 0.070 dps/LSB, accel 0.976 mg/LSB  (±2000dps / ±32g)
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

#pragma pack(pop)

#endif
