#include "MMC5983MA.h"

MMC5983MA::MMC5983MA(TwoWire* wire, uint8_t i2cAddr) {
    _wire    = wire;
    _i2cAddr = i2cAddr;
}

bool MMC5983MA::begin() {
    if (!isConnected()) return false;

    // Soft reset returns the chip to a known state. All control registers = 0.
    softReset();

    // Apply ES-EKF attitude config (see DEFAULT_* constants in header).
    applyDefaults();

    return true;
}

void MMC5983MA::applyDefaults() {
    setFilterBandwidth(DEFAULT_BW_HZ);
    enableAutoSetReset(true);
    setPeriodicSetCount(DEFAULT_PRD_SET_CNT);
    enablePeriodicSet(true);
    setContinuousFrequency(DEFAULT_CMM_HZ);
    enableContinuousMode(true);
}

// ==================== Settings ====================

bool MMC5983MA::setFilterBandwidth(uint16_t bw) {
    // Clear BW bits in shadow first (no write)
    shadowClear(REG_CTRL1, BW0 | BW1, false);

    switch (bw) {
        case 800: shadowSet(REG_CTRL1, BW0 | BW1); break;
        case 400: shadowSet(REG_CTRL1, BW1);        break;
        case 200: shadowSet(REG_CTRL1, BW0);        break;
        case 100: writeRegister(REG_CTRL1, _shadowCtrl1); break; // already cleared
        default:  return false;
    }
    return true;
}

bool MMC5983MA::setContinuousFrequency(uint16_t freq) {
    // Clear CM_FREQ bits (no write)
    shadowClear(REG_CTRL2, CM_FREQ_0 | CM_FREQ_1 | CM_FREQ_2, false);

    uint8_t val = 0;
    switch (freq) {
        case 0:    val = 0; break;
        case 1:    val = 1; break;
        case 10:   val = 2; break;
        case 20:   val = 3; break;
        case 50:   val = 4; break;
        case 100:  val = 5; break;
        case 200:  val = 6; break;
        case 1000: val = 7; break;
        default:   return false;
    }

    shadowSet(REG_CTRL2, val & 0x07);
    return true;
}

void MMC5983MA::enableAutoSetReset(bool enable) {
    if (enable)
        shadowSet(REG_CTRL0, AUTO_SR_EN);
    else
        shadowClear(REG_CTRL0, AUTO_SR_EN);
}

void MMC5983MA::enableContinuousMode(bool enable) {
    if (enable)
        shadowSet(REG_CTRL2, CMM_EN);
    else
        shadowClear(REG_CTRL2, CMM_EN);
}

bool MMC5983MA::setPeriodicSetCount(uint16_t samples) {
    // Clear PRD_SET bits (no write)
    shadowClear(REG_CTRL2, PRD_SET_0 | PRD_SET_1 | PRD_SET_2, false);

    uint8_t val = 0;
    switch (samples) {
        case 1:    val = 0; break;
        case 25:   val = 1; break;
        case 75:   val = 2; break;
        case 100:  val = 3; break;
        case 250:  val = 4; break;
        case 500:  val = 5; break;
        case 1000: val = 6; break;
        case 2000: val = 7; break;
        default:   return false;
    }

    shadowSet(REG_CTRL2, (val & 0x07) << 4);
    return true;
}

void MMC5983MA::enablePeriodicSet(bool enable) {
    if (enable)
        shadowSet(REG_CTRL2, EN_PRD_SET);
    else
        shadowClear(REG_CTRL2, EN_PRD_SET);
}

void MMC5983MA::applyConfig(const MMC5983MA_Config& cfg) {
    setFilterBandwidth(cfg.bandwidth);
    enableAutoSetReset(cfg.autoSetReset);
    setContinuousFrequency(cfg.cmFrequency);
    enableContinuousMode(cfg.continuousMode);
}

MMC5983MA_Config MMC5983MA::getConfig() {
    MMC5983MA_Config cfg;

    // Bandwidth from shadow
    uint8_t bwBits = _shadowCtrl1 & (BW0 | BW1);
    switch (bwBits) {
        case 0x03: cfg.bandwidth = 800; break;
        case 0x02: cfg.bandwidth = 400; break;
        case 0x01: cfg.bandwidth = 200; break;
        default:   cfg.bandwidth = 100; break;
    }

    // Continuous frequency from shadow
    uint8_t cmBits = _shadowCtrl2 & 0x07;
    const uint16_t freqTable[] = {0, 1, 10, 20, 50, 100, 200, 1000};
    cfg.cmFrequency = freqTable[cmBits];

    cfg.autoSetReset  = (_shadowCtrl0 & AUTO_SR_EN) != 0;
    cfg.continuousMode = (_shadowCtrl2 & CMM_EN) != 0;

    return cfg;
}

// ==================== Raw Data ====================

void MMC5983MA::readRawMag(uint32_t &mx, uint32_t &my, uint32_t &mz) {
    uint8_t buf[7];
    readRegisters(REG_X_OUT_0, buf, 7);

    mx = (uint32_t)buf[0] << 10 | (uint32_t)buf[1] << 2 | (buf[6] >> 6);
    my = (uint32_t)buf[2] << 10 | (uint32_t)buf[3] << 2 | ((buf[6] >> 4) & 0x03);
    mz = (uint32_t)buf[4] << 10 | (uint32_t)buf[5] << 2 | ((buf[6] >> 2) & 0x03);
}

void MMC5983MA::readMag(float &mx, float &my, float &mz) {
    uint32_t rx, ry, rz;
    readRawMag(rx, ry, rz);
    // 18-bit offset binary: null = 131072, sensitivity = 16384 counts/Gauss
    mx = ((float)rx - 131072.0f) / 16384.0f;
    my = ((float)ry - 131072.0f) / 16384.0f;
    mz = ((float)rz - 131072.0f) / 16384.0f;
}

// ==================== Calibration ====================

bool MMC5983MA::calibrate(uint32_t durationMs) {
    Serial.println();
    Serial.println("=== Magnetometer Calibration ===");
    Serial.printf("Rotate the board in ALL orientations for %lu s (figure-8).\n",
                  durationMs / 1000);
    Serial.println("Start in 3..."); delay(1000);
    Serial.println("      2...");    delay(1000);
    Serial.println("      1...");    delay(1000);
    Serial.println("GO!");

    // Boost CMM rate during calibration; applyDefaults() restores afterwards.
    setContinuousFrequency(1000);
    clearInterruptFlag();

    float minX =  1e9f, maxX = -1e9f;
    float minY =  1e9f, maxY = -1e9f;
    float minZ =  1e9f, maxZ = -1e9f;
    uint32_t samples = 0;

    const uint32_t start = millis();
    uint32_t lastReport  = start;

    while (millis() - start < durationMs) {
        const uint32_t t0 = millis();
        while (!isDataReady()) {
            if (millis() - t0 > 50) break;
        }
        if (!isDataReady()) continue;

        float mx, my, mz;
        readMag(mx, my, mz);
        clearInterruptFlag();

        if (mx < minX) minX = mx;  if (mx > maxX) maxX = mx;
        if (my < minY) minY = my;  if (my > maxY) maxY = my;
        if (mz < minZ) minZ = mz;  if (mz > maxZ) maxZ = mz;
        samples++;

        if (millis() - lastReport >= 1000) {
            lastReport = millis();
            uint32_t remain = (durationMs - (millis() - start)) / 1000;
            Serial.printf("  [%2lus] X[%+.3f,%+.3f] Y[%+.3f,%+.3f] Z[%+.3f,%+.3f]\n",
                          remain, minX, maxX, minY, maxY, minZ, maxZ);
        }
    }

    // 품질 검증: 충분히 회전했는지 확인 (범위가 0.3 Gauss 미만이면 불충분)
    float spreadX = maxX - minX;
    float spreadY = maxY - minY;
    float spreadZ = maxZ - minZ;
    if (spreadX < 0.3f || spreadY < 0.3f || spreadZ < 0.3f) {
        Serial.printf("Mag 교정 실패: 회전 부족 (X:%.2f, Y:%.2f, Z:%.2f)\n", spreadX, spreadY, spreadZ);
        applyDefaults();
        return false;
    }

    // Hard-iron: midpoint of min/max per axis
    _bias[0] = 0.5f * (maxX + minX);
    _bias[1] = 0.5f * (maxY + minY);
    _bias[2] = 0.5f * (maxZ + minZ);

    // Soft-iron: normalize each axis radius to mean radius
    float rX = 0.5f * (maxX - minX);
    float rY = 0.5f * (maxY - minY);
    float rZ = 0.5f * (maxZ - minZ);
    float rAvg = (rX + rY + rZ) / 3.0f;
    _scale[0] = (rX > 1e-6f) ? (rAvg / rX) : 1.0f;
    _scale[1] = (rY > 1e-6f) ? (rAvg / rY) : 1.0f;
    _scale[2] = (rZ > 1e-6f) ? (rAvg / rZ) : 1.0f;

    applyDefaults();
    clearInterruptFlag();

    Serial.println("=== Calibration complete ===");
    Serial.printf("  samples : %lu\n", samples);
    Serial.printf("  bias  (Gauss): %+.4f  %+.4f  %+.4f\n",
                  _bias[0], _bias[1], _bias[2]);
    Serial.printf("  scale        : %.4f  %.4f  %.4f\n",
                  _scale[0], _scale[1], _scale[2]);
    return true;
}

bool MMC5983MA::readCalibratedMag(float &mx, float &my, float &mz) {
    float sx, sy, sz;
    readMag(sx, sy, sz);

    // 1. Hard-iron + soft-iron calibration (in sensor frame)
    sx = (sx - _bias[0]) * _scale[0];
    sy = (sy - _bias[1]) * _scale[1];
    sz = (sz - _bias[2]) * _scale[2];

    // 2. Align to Rocket Body-Axis (align_axis logic)
    // Body X = Sensor X (Nosecone)
    // Body Y = -Sensor Y (Sensor Y was Left, make it Right)
    // Body Z = -Sensor Z (Sensor Z was Up, make it Down for RHR)
    mx =  sx;
    my = -sy;
    mz = -sz;
    return true;
}

// ==================== Interrupt ====================

void MMC5983MA::enableDataReadyInterrupt(bool enable) {
    if (enable)
        shadowSet(REG_CTRL0, INT_MEAS_DONE_EN);
    else
        shadowClear(REG_CTRL0, INT_MEAS_DONE_EN);
}

void MMC5983MA::clearInterruptFlag() {
    // Meas_M_Done is write-1-to-clear; clears the INT pin so a new rising edge can fire
    writeRegister(REG_STATUS, MEAS_M_DONE);
}

bool MMC5983MA::isDataReady() {
    return (readRegister(REG_STATUS) & MEAS_M_DONE) != 0;
}

void MMC5983MA::triggerSingleMeasurement() {
    // Clear Meas_M_Done (W1C) so we can wait for the next completion cleanly.
    writeRegister(REG_STATUS, MEAS_M_DONE);
    // TM_M is self-clearing. Write bare TM_M so we do NOT re-enable AUTO_SR
    // during a one-shot — AUTO_SR+TM_M only does a SET pulse, leaving a large
    // internal-field bias on the reading.
    // NOTE: TM_M is ignored while CMM_EN=1, so caller must disable continuous mode.
    writeRegister(REG_CTRL0, TM_M);
}

bool MMC5983MA::readMagSingleShot(uint32_t& mx, uint32_t& my, uint32_t& mz,
                                  uint32_t timeoutMs) {
    triggerSingleMeasurement();

    // Poll for completion (measurement ~1.25 ms at 800Hz BW, ~10 ms at 100Hz)
    const uint32_t start = millis();
    while ((readRegister(REG_STATUS) & MEAS_M_DONE) == 0) {
        if (millis() - start > timeoutMs) return false;
        delayMicroseconds(200);
    }

    readRawMag(mx, my, mz);
    return true;
}

// ==================== Utility ====================

bool MMC5983MA::isConnected() {
    uint8_t id = readRegister(REG_PROD_ID);
    return (id == PROD_ID_VAL);
}

void MMC5983MA::softReset() {
    shadowSet(REG_CTRL1, SW_RST);
    // Reset clears all registers - reset shadows
    _shadowCtrl0 = 0;
    _shadowCtrl1 = 0;
    _shadowCtrl2 = 0;
    _shadowCtrl3 = 0;
    delay(15);
}

void MMC5983MA::performSetOperation() {
    shadowSet(REG_CTRL0, SET_OPERATION);
    shadowClear(REG_CTRL0, SET_OPERATION, false); // auto-clear in shadow only
    delay(1);
}

void MMC5983MA::performResetOperation() {
    shadowSet(REG_CTRL0, RESET_OPERATION);
    shadowClear(REG_CTRL0, RESET_OPERATION, false); // auto-clear in shadow only
    delay(1);
}

// ==================== Shadow Register Helpers ====================

void MMC5983MA::shadowSet(uint8_t reg, uint8_t mask, bool write) {
    uint8_t* shadow = nullptr;
    switch (reg) {
        case REG_CTRL0: shadow = &_shadowCtrl0; break;
        case REG_CTRL1: shadow = &_shadowCtrl1; break;
        case REG_CTRL2: shadow = &_shadowCtrl2; break;
        case REG_CTRL3: shadow = &_shadowCtrl3; break;
        default: return;
    }
    *shadow |= mask;
    if (write) writeRegister(reg, *shadow);
}

void MMC5983MA::shadowClear(uint8_t reg, uint8_t mask, bool write) {
    uint8_t* shadow = nullptr;
    switch (reg) {
        case REG_CTRL0: shadow = &_shadowCtrl0; break;
        case REG_CTRL1: shadow = &_shadowCtrl1; break;
        case REG_CTRL2: shadow = &_shadowCtrl2; break;
        case REG_CTRL3: shadow = &_shadowCtrl3; break;
        default: return;
    }
    *shadow &= ~mask;
    if (write) writeRegister(reg, *shadow);
}

// ==================== I2C Low-Level ====================

uint8_t MMC5983MA::readRegister(uint8_t reg) {
    _wire->beginTransmission(_i2cAddr);
    _wire->write(reg);
    if (_wire->endTransmission() != 0) return 0;  // STOP, not repeated-start

    if (_wire->requestFrom((int)_i2cAddr, 1) != 1) return 0;
    return _wire->read();
}

void MMC5983MA::readRegisters(uint8_t reg, uint8_t* buffer, uint8_t len) {
    _wire->beginTransmission(_i2cAddr);
    _wire->write(reg);
    if (_wire->endTransmission() != 0) return;  // STOP, not repeated-start

    uint8_t received = _wire->requestFrom((int)_i2cAddr, (int)len);
    for (uint8_t i = 0; i < received && i < len; i++) {
        buffer[i] = _wire->read();
    }
}

void MMC5983MA::writeRegister(uint8_t reg, uint8_t data) {
    _wire->beginTransmission(_i2cAddr);
    _wire->write(reg);
    _wire->write(data);
    _wire->endTransmission();
}