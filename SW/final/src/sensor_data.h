#ifndef SENSOR_DATA_H
#define SENSOR_DATA_H

#include <stdint.h>

#pragma pack(push, 1)

struct PacketHeader {
  uint8_t SYNC_BYTE = 0xAA;
  uint8_t id;   
  uint8_t len;  
};
struct baro_pkt {
  PacketHeader header;
  uint32_t t;          
  float rawalt;        
};
struct imu_pkt {
  PacketHeader header;
  uint32_t t;          
  int16_t rawacc_x;    
  int16_t rawacc_y;
  int16_t rawacc_z;
  int16_t rawgyro_x;   
  int16_t rawgyro_y;
  int16_t rawgyro_z;
};

#pragma pack(pop)

#endif
