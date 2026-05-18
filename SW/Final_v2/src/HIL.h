#ifndef HIL_H
#define HIL_H

#include <Arduino.h>
#include "sensor_data.h"

// ============================================================
//  HIL — Push-mode: MATLAB streams [ID][Payload] packets.
//
//  MATLAB sends:
//    IMU  packet: 0x01 + 12 bytes (gx,gy,gz,ax,ay,az — int16 LE)
//    BARO packet: 0x02 + 4  bytes (alt m — float32 LE)
//    DONE mark:   0xFF
//
//  Firmware (HIL_Task in main.cpp) receives packets and feeds them
//  through the NAV / KF / logger pipeline.
// ============================================================

class HIL {
public:
    static constexpr uint8_t  CMD_IMU      = 0x01;
    static constexpr uint8_t  CMD_BARO     = 0x02;
    static constexpr uint8_t  CMD_DONE     = 0xFD;
    static constexpr uint32_t IMU_DT_US   = 2403;   // 1/416 s

    bool loadHeader();             // receive 8-byte header [imu_count, bmp_count]
    void sendDone();               // send "HIL DONE\n" when finished
    bool readBytesBlocking(uint8_t *dst, size_t n, uint32_t timeout_ms = 5000);

    bool isHeaderLoaded() const { return _imu_count > 0 && _bmp_count > 0; }
    bool isActive()       const { return _active; }
    void setActive(bool a)      { _active = a; }

    uint32_t imuCount() const { return _imu_count; }
    uint32_t bmpCount() const { return _bmp_count; }
    void reset();

private:
    uint32_t _imu_count = 0, _bmp_count = 0;
    volatile bool _active = false;
};

extern HIL hil;

#endif
