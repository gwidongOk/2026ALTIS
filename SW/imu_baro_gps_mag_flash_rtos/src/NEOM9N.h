#pragma once
#include <Arduino.h>

// Raw GPS data from NAV-PVT
struct GPS_Raw {
    uint32_t iTOW;     // GPS time of week [ms]
    int32_t  lat;      // [deg * 1e-7]
    int32_t  lon;      // [deg * 1e-7]
    int32_t  alt;      // height above ellipsoid [mm]
    int32_t  velN;     // NED north velocity [mm/s]
    int32_t  velE;     // NED east velocity [mm/s]
    int32_t  velD;     // NED down velocity [mm/s]
    int32_t  gSpeed;   // ground speed [mm/s]
    int32_t  headMot;  // heading of motion [deg * 1e-5]
    uint32_t hAcc;     // horizontal accuracy [mm]
    uint32_t vAcc;     // vertical accuracy [mm]
    uint32_t sAcc;     // speed accuracy [mm/s]
    uint32_t headAcc;  // heading accuracy [deg * 1e-5]
    uint8_t  fixType;
    uint8_t  numSV;
    bool     fresh;
};

class NEOM9N {
public:
    NEOM9N(HardwareSerial &serial, int rxPin, int txPin);

    bool begin(uint16_t rateHz = 10);
    bool update();

    GPS_Raw getRaw() const;
    bool hasFix() const;

    // 편의 함수 — 단위 변환된 값 반환
    float getLatDeg() const;          // [deg]
    float getLonDeg() const;          // [deg]
    float getAltM() const;            // [m]
    float getVelN() const;            // [m/s]
    float getVelE() const;            // [m/s]
    float getVelD() const;            // [m/s]
    float getGroundSpeed() const;     // [m/s]
    float getHeading() const;         // heading of motion [deg]
    float getHeadingAcc() const;      // heading accuracy [deg]
    float getSpeedAcc() const;        // speed accuracy [m/s]
    float getHorizontalAcc() const;   // [m]
    float getVerticalAcc() const;     // [m]

    // ===== Calibration (set local NED origin) =====
    // Uses current fix. Returns false if fix<3 or hAcc too poor.
    bool calibrate(float maxHAcc_m = 5.0f);
    bool isOriginSet() const { return _origin_set; }

    // Get NED position (m) + velocity (m/s) + accuracy (m) + fix info.
    // Returns false if no origin set or no 3D fix yet (pn/pe/pd = 0 in that case).
    bool getNED(float &pn, float &pe, float &pd,
                float &vn, float &ve, float &vd,
                float &hAcc, float &vAcc,
                uint8_t &fixType, uint8_t &numSV);

private:
    HardwareSerial &_serial;
    int _rxPin, _txPin;
    GPS_Raw _pvt;

    // NED origin (double for precision; single cos cached)
    bool   _origin_set = false;
    double _origin_lat_rad = 0.0;
    double _origin_lon_rad = 0.0;
    double _origin_alt_m   = 0.0;
    double _cos_origin_lat = 1.0;

    // UBX parser
    enum State : uint8_t {
        SYNC1, SYNC2, CLASS, ID, LEN1, LEN2, PAYLOAD, CKA, CKB
    };
    State    _ps;
    uint8_t  _msgCls, _msgId;
    uint16_t _pLen, _pIdx;
    uint8_t  _buf[100];
    uint8_t  _ckA, _ckB;
    volatile bool _gotACK, _gotNAK;

    void feed(uint8_t b);
    void dispatch();
    void parseNavPVT();

    void sendUBX(uint8_t cls, uint8_t id, const uint8_t *pl, uint16_t len);
    bool waitACK(uint8_t cls, uint8_t id, uint32_t ms = 1000);
    void drain();
    bool probe();

    bool setUART1_UBXOnly();
    bool setRate(uint16_t hz);
    bool setDynModel(uint8_t model);
    bool disableNMEA();
    bool enableNavPVT();
    bool saveCfg();
};
