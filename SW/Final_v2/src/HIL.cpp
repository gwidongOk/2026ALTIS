#include "HIL.h"

HIL hil;

bool HIL::readBytesBlocking(uint8_t *dst, size_t n, uint32_t timeout_ms) {
    size_t got = 0;
    uint32_t t0 = millis();
    while (got < n) {
        int avail = Serial.available();
        if (avail > 0) {
            size_t want = (size_t)avail;
            if (want > n - got) want = n - got;
            got += Serial.readBytes(dst + got, want);
            t0 = millis();
        } else if (millis() - t0 > timeout_ms) {
            return false;
        }
    }
    return true;
}

bool HIL::loadHeader() {
    uint8_t hdr[8];
    if (!readBytesBlocking(hdr, sizeof(hdr), 10000)) {
        Serial.println("HIL: header timeout");
        return false;
    }
    memcpy(&_imu_count, hdr,     4);
    memcpy(&_bmp_count, hdr + 4, 4);
    if (_imu_count == 0 || _bmp_count == 0 ||
        _imu_count > 2000000 || _bmp_count > 200000) {
        Serial.printf("HIL: bad counts imu=%u bmp=%u\n", _imu_count, _bmp_count);
        _imu_count = _bmp_count = 0;
        return false;
    }
    Serial.printf("HIL: header ok imu=%u bmp=%u\n", _imu_count, _bmp_count);
    return true;
}

void HIL::sendDone() {
    Serial.write(CMD_DONE);
    Serial.print("\nHIL DONE\n");
    Serial.flush();
}

void HIL::reset() {
    _imu_count = _bmp_count = 0;
    _active = false;
}
