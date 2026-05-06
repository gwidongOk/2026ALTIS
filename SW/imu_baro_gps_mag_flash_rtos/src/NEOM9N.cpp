#include "NEOM9N.h"
#include <math.h>

static constexpr double R_EARTH = 6378137.0;
static constexpr double DEG2RAD = M_PI / 180.0;

#define UBX_SYNC1   0xB5
#define UBX_SYNC2   0x62
#define CLS_NAV     0x01
#define CLS_ACK     0x05
#define CLS_CFG     0x06
#define ID_NAV_PVT  0x07
#define ID_ACK_ACK  0x01
#define ID_ACK_NAK  0x00
#define ID_CFG_PRT  0x00
#define ID_CFG_MSG  0x01
#define ID_CFG_RATE 0x08
#define ID_CFG_NAV5 0x24
#define ID_CFG_CFG  0x09

NEOM9N::NEOM9N(HardwareSerial &serial, int rxPin, int txPin)
    : _serial(serial), _rxPin(rxPin), _txPin(txPin),
      _ps(SYNC1), _pLen(0), _pIdx(0), _ckA(0), _ckB(0),
      _gotACK(false), _gotNAK(false)
{
    memset(&_pvt, 0, sizeof(_pvt));
}

// ── Public ──────────────────────────────────────────────

bool NEOM9N::begin(uint16_t rateHz) {
    const uint32_t bauds[] = {38400, 9600, 115200, 57600};
    bool connected = false;

    for (auto baud : bauds) {
        Serial.printf("[GPS] Trying %lu baud...\n", baud);
        _serial.begin(baud, SERIAL_8N1, _rxPin, _txPin);
        delay(300);
        drain();

        if (probe()) {
            Serial.printf("[GPS] Connected at %lu baud\n", baud);
            connected = true;
            break;
        }
        _serial.end();
        delay(100);
    }
    if (!connected) return false;

    setUART1_UBXOnly();
    delay(100);
    disableNMEA();
    enableNavPVT();
    setRate(rateHz);
    setDynModel(8);  // Airborne <4g
    saveCfg();

    Serial.printf("[GPS] NEO-M9N configured: %dHz, Airborne<4g\n", rateHz);
    return true;
}

bool NEOM9N::update() {
    _pvt.fresh = false;
    while (_serial.available()) {
        feed(_serial.read());
    }
    return _pvt.fresh;
}

GPS_Raw NEOM9N::getRaw() const { return _pvt; }
bool NEOM9N::hasFix() const { return _pvt.fixType >= 3; }

float NEOM9N::getLatDeg() const        { return _pvt.lat * 1e-7f; }
float NEOM9N::getLonDeg() const        { return _pvt.lon * 1e-7f; }
float NEOM9N::getAltM() const          { return _pvt.alt * 0.001f; }
float NEOM9N::getVelN() const          { return _pvt.velN * 0.001f; }
float NEOM9N::getVelE() const          { return _pvt.velE * 0.001f; }
float NEOM9N::getVelD() const          { return _pvt.velD * 0.001f; }
float NEOM9N::getGroundSpeed() const   { return _pvt.gSpeed * 0.001f; }
float NEOM9N::getHeading() const       { return _pvt.headMot * 1e-5f; }
float NEOM9N::getHeadingAcc() const    { return _pvt.headAcc * 1e-5f; }
float NEOM9N::getSpeedAcc() const      { return _pvt.sAcc * 0.001f; }
float NEOM9N::getHorizontalAcc() const { return _pvt.hAcc * 0.001f; }
float NEOM9N::getVerticalAcc() const   { return _pvt.vAcc * 0.001f; }

// ── Calibration (NED origin) ────────────────────────────

bool NEOM9N::calibrate(float maxHAcc_m) {
    if (_pvt.fixType < 3) {
        Serial.println("[GPS] calibrate: no 3D fix");
        return false;
    }

    const int nSamples = 20;
    double latSum = 0, lonSum = 0, altSum = 0;
    double latStart = _pvt.lat * 1e-7, lonStart = _pvt.lon * 1e-7;
    int collected = 0;

    Serial.println("[GPS] Averaging origin position...");
    uint32_t t0 = millis();
    while (collected < nSamples && millis() - t0 < 10000) {
        if (update()) {
            // 안정성 체크 1: 현재 속도가 너무 빠르면 (드리프트가 심하면) 실패
            float speed = _pvt.gSpeed * 0.001f;
            if (speed > 0.5f) {
                Serial.printf("[GPS] Calib fail: moving too fast (%.2f m/s)\n", speed);
                return false;
            }

            // 안정성 체크 2: 처음 지점으로부터 너무 멀어지면 실패
            double latNow = _pvt.lat * 1e-7;
            double lonNow = _pvt.lon * 1e-7;
            double dLat = (latNow - latStart) * DEG2RAD * R_EARTH;
            double dLon = (lonNow - lonStart) * DEG2RAD * R_EARTH * cos(latStart * DEG2RAD);
            if (sqrt(dLat*dLat + dLon*dLon) > 5.0) {
                Serial.println("[GPS] Calib fail: position drifting too much");
                return false;
            }

            latSum += latNow;
            lonSum += lonNow;
            altSum += _pvt.alt * 1e-3;
            collected++;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    if (collected < nSamples) return false;

    _origin_lat_rad = (latSum / collected) * DEG2RAD;
    _origin_lon_rad = (lonSum / collected) * DEG2RAD;
    _origin_alt_m   = (altSum / collected);
    _cos_origin_lat = cos(_origin_lat_rad);
    _origin_set = true;

    Serial.printf("[GPS] Origin set (Avg): lat=%.7f lon=%.7f alt=%.2f m\n",
                  _origin_lat_rad/DEG2RAD, _origin_lon_rad/DEG2RAD, _origin_alt_m);
    return true;
}

bool NEOM9N::getNED(float &pn, float &pe, float &pd,
                    float &vn, float &ve, float &vd,
                    float &hAcc, float &vAcc,
                    uint8_t &fixType, uint8_t &numSV) {
    fixType = _pvt.fixType;
    numSV   = _pvt.numSV;
    hAcc    = _pvt.hAcc * 1e-3f;
    vAcc    = _pvt.vAcc * 1e-3f;

    vn = _pvt.velN * 1e-3f;
    ve = _pvt.velE * 1e-3f;
    vd = _pvt.velD * 1e-3f;

    if (!_origin_set || _pvt.fixType < 3) {
        pn = pe = pd = 0.0f;
        return false;
    }

    double lat_rad = _pvt.lat * 1e-7 * DEG2RAD;
    double lon_rad = _pvt.lon * 1e-7 * DEG2RAD;
    double alt_m   = _pvt.alt * 1e-3;

    pn = (float)((lat_rad - _origin_lat_rad) * R_EARTH);
    pe = (float)((lon_rad - _origin_lon_rad) * R_EARTH * _cos_origin_lat);
    pd = (float)(-(alt_m - _origin_alt_m));
    return true;
}

// ── UBX Parser ──────────────────────────────────────────

void NEOM9N::feed(uint8_t b) {
    switch (_ps) {
    case SYNC1:   if (b == UBX_SYNC1) _ps = SYNC2; return;
    case SYNC2:   _ps = (b == UBX_SYNC2) ? CLASS : SYNC1; return;
    case CLASS:   _msgCls = b; _ckA = b; _ckB = b; _ps = ID; return;
    case ID:      _msgId = b; _ckA += b; _ckB += _ckA; _ps = LEN1; return;
    case LEN1:    _pLen = b; _ckA += b; _ckB += _ckA; _ps = LEN2; return;
    case LEN2:
        _pLen |= (uint16_t)b << 8;
        _ckA += b; _ckB += _ckA;
        _pIdx = 0;
        if (_pLen == 0) { _ps = CKA; return; }
        _ps = (_pLen <= sizeof(_buf)) ? PAYLOAD : SYNC1;
        return;
    case PAYLOAD:
        _buf[_pIdx++] = b;
        _ckA += b; _ckB += _ckA;
        if (_pIdx >= _pLen) _ps = CKA;
        return;
    case CKA:     _ps = (b == _ckA) ? CKB : SYNC1; return;
    case CKB:
        if (b == _ckB) dispatch();
        _ps = SYNC1;
        return;
    }
}

void NEOM9N::dispatch() {
    if (_msgCls == CLS_NAV && _msgId == ID_NAV_PVT) parseNavPVT();
    else if (_msgCls == CLS_ACK) {
        if (_msgId == ID_ACK_ACK) _gotACK = true;
        else if (_msgId == ID_ACK_NAK) _gotNAK = true;
    }
}

void NEOM9N::parseNavPVT() {
    if (_pLen < 92) return;
    _pvt.iTOW    = *(uint32_t *)&_buf[0];
    _pvt.fixType = _buf[20];
    _pvt.numSV   = _buf[23];
    _pvt.lon     = *(int32_t *)&_buf[24];
    _pvt.lat     = *(int32_t *)&_buf[28];
    _pvt.alt     = *(int32_t *)&_buf[32];
    _pvt.hAcc    = *(uint32_t *)&_buf[40];
    _pvt.vAcc    = *(uint32_t *)&_buf[44];
    _pvt.velN    = *(int32_t *)&_buf[48];
    _pvt.velE    = *(int32_t *)&_buf[52];
    _pvt.velD    = *(int32_t *)&_buf[56];
    _pvt.gSpeed  = *(int32_t *)&_buf[60];
    _pvt.headMot = *(int32_t *)&_buf[64];
    _pvt.sAcc    = *(uint32_t *)&_buf[68];
    _pvt.headAcc = *(uint32_t *)&_buf[72];
    _pvt.fresh   = true;
}

// ── UBX Send / ACK ──────────────────────────────────────

void NEOM9N::sendUBX(uint8_t cls, uint8_t id, const uint8_t *pl, uint16_t len) {
    uint8_t a = 0, b = 0;
    auto ck = [&](uint8_t v) { a += v; b += a; };

    _serial.write(UBX_SYNC1);
    _serial.write(UBX_SYNC2);
    _serial.write(cls);  ck(cls);
    _serial.write(id);   ck(id);
    _serial.write((uint8_t)(len & 0xFF));        ck(len & 0xFF);
    _serial.write((uint8_t)((len >> 8) & 0xFF)); ck((len >> 8) & 0xFF);
    for (uint16_t i = 0; i < len; i++) {
        _serial.write(pl[i]); ck(pl[i]);
    }
    _serial.write(a);
    _serial.write(b);
    _serial.flush();
}

bool NEOM9N::waitACK(uint8_t cls, uint8_t id, uint32_t ms) {
    _gotACK = _gotNAK = false;
    uint32_t t0 = millis();
    while (millis() - t0 < ms) {
        while (_serial.available()) feed(_serial.read());
        if (_gotACK) return true;
        if (_gotNAK) return false;
        delay(1);
    }
    return false;
}

void NEOM9N::drain() {
    while (_serial.available()) _serial.read();
    _ps = SYNC1;
}

bool NEOM9N::probe() {
    sendUBX(CLS_CFG, ID_CFG_PRT, nullptr, 0);
    uint32_t t0 = millis();
    while (millis() - t0 < 1500) {
        while (_serial.available()) {
            feed(_serial.read());
            if (_gotACK) return true;
        }
        delay(1);
    }
    drain();
    t0 = millis();
    while (millis() - t0 < 1500) {
        if (_serial.available()) {
            uint8_t c = _serial.read();
            if (c == UBX_SYNC1 || c == '$') return true;
        }
        delay(1);
    }
    return false;
}

// ── Configuration ───────────────────────────────────────

bool NEOM9N::setUART1_UBXOnly() {
    uint8_t msg[20] = {};
    msg[0] = 1; msg[4] = 0xC0; msg[5] = 0x08;
    uint32_t baud = 38400;
    msg[8] = baud; msg[9] = baud >> 8; msg[10] = baud >> 16; msg[11] = baud >> 24;
    msg[12] = 0x01; msg[14] = 0x01;
    sendUBX(CLS_CFG, ID_CFG_PRT, msg, sizeof(msg));
    delay(100);
    return true;
}

bool NEOM9N::disableNMEA() {
    const uint8_t ids[] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05};
    for (auto id : ids) {
        uint8_t msg[8] = {};
        msg[0] = 0xF0; msg[1] = id;
        sendUBX(CLS_CFG, ID_CFG_MSG, msg, sizeof(msg));
        waitACK(CLS_CFG, ID_CFG_MSG, 300);
        delay(20);
    }
    return true;
}

bool NEOM9N::enableNavPVT() {
    uint8_t msg[8] = {};
    msg[0] = CLS_NAV; msg[1] = ID_NAV_PVT; msg[3] = 1;
    sendUBX(CLS_CFG, ID_CFG_MSG, msg, sizeof(msg));
    return waitACK(CLS_CFG, ID_CFG_MSG);
}

bool NEOM9N::setRate(uint16_t hz) {
    uint16_t ms = 1000 / hz;
    uint8_t msg[6] = { (uint8_t)(ms & 0xFF), (uint8_t)(ms >> 8), 1, 0, 1, 0 };
    sendUBX(CLS_CFG, ID_CFG_RATE, msg, sizeof(msg));
    return waitACK(CLS_CFG, ID_CFG_RATE);
}

bool NEOM9N::setDynModel(uint8_t model) {
    uint8_t msg[36] = {};
    msg[0] = 0x01; msg[2] = model;
    sendUBX(CLS_CFG, ID_CFG_NAV5, msg, sizeof(msg));
    return waitACK(CLS_CFG, ID_CFG_NAV5);
}

bool NEOM9N::saveCfg() {
    uint8_t msg[13] = {};
    msg[4] = 0x1F; msg[8] = 0x17;
    sendUBX(CLS_CFG, ID_CFG_CFG, msg, sizeof(msg));
    return waitACK(CLS_CFG, ID_CFG_CFG);
}
