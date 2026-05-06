#ifndef SENSOR_DATA_H
#define SENSOR_DATA_H

#include <stdint.h>

#pragma pack(push, 1)

//공용 헤더 구조체 정의
struct PacketHeader {
  uint8_t SYNC_BYTE = 0xAA;  // 0xAA
  uint8_t id;   // 센서 종류 (0~255)
  uint8_t len;  // 이 패킷의 전체 크기 (헤더 2바이트 포함, 최대 255바이트)
};

struct baro {
  PacketHeader header; // 헤더 (2 bytes)
  uint32_t t;          // 시간 (4 bytes)
  float rawalt;        // 고도 (4 bytes)
};

struct imu {
  PacketHeader header; // 헤더 (2 bytes)
  uint32_t t;          // 시간 (4 bytes)
  uint16_t rawacc_x;   // (2 bytes)
  uint16_t rawacc_y;   // (2 bytes)
  uint16_t rawacc_z;   // (2 bytes)
  uint16_t rawgyro_x;  // (2 bytes)
  uint16_t rawgyro_y;  // (2 bytes)
  uint16_t rawgyro_z;  // (2 bytes)
};

#pragma pack(pop)

#endif