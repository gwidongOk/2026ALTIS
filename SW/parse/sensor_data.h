#ifndef PYTHON_HEADER_H
#define PYTHON_HEADER_H

#include <stdint.h>

// ============================================================
// Packet IDs
// ============================================================
#define ID_BARO  1
#define ID_IMU   2
#define ID_STATE 5
#define ID_EVENT 6

#pragma pack(push, 1)

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

// ID 5 : Navigation state (NED p/v + quaternion)
struct state_pkt {
  PacketHeader header;
  uint32_t t;
  float pN;
  float pE;
  float pD;
  float vN;
  float vE;
  float vD;
  float qw;
  float qx;
  float qy;
  float qz;
};

// ID 6 : Flight event
struct event_pkt {
  PacketHeader header;
  uint32_t t;
  uint8_t  phase;
  uint8_t  event_id;
};

#pragma pack(pop)

#endif
