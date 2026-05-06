#ifndef MMC5983MA_H
#define MMC5983MA_H

#include <Arduino.h>
#include <Wire.h>

struct MMC5983MA_Config {
    uint16_t bandwidth;       // 100, 200, 400, 800 (Hz)
    uint16_t cmFrequency;     // Continuous mode: 0(off), 1, 10, 20, 50, 100, 200, 1000 (Hz)
    bool     autoSetReset;    // Auto SET/RESET enable
    bool     continuousMode;  // Continuous measurement enable
};

class MMC5983MA {
private:
    TwoWire* _wire;
    uint8_t  _i2cAddr;

    // I2C 7-bit address (SA0 tied low on the module; datasheet default)
    static const uint8_t I2C_ADDR_DEFAULT = 0x30;

    // --- Register addresses ---
    static const uint8_t REG_X_OUT_0    = 0x00;
    static const uint8_t REG_X_OUT_1    = 0x01;
    static const uint8_t REG_Y_OUT_0    = 0x02;
    static const uint8_t REG_Y_OUT_1    = 0x03;
    static const uint8_t REG_Z_OUT_0    = 0x04;
    static const uint8_t REG_Z_OUT_1    = 0x05;
    static const uint8_t REG_XYZ_OUT_2  = 0x06;
    static const uint8_t REG_T_OUT      = 0x07;
    static const uint8_t REG_STATUS     = 0x08;
    static const uint8_t REG_CTRL0      = 0x09;
    static const uint8_t REG_CTRL1      = 0x0A;
    static const uint8_t REG_CTRL2      = 0x0B;
    static const uint8_t REG_CTRL3      = 0x0C;
    static const uint8_t REG_PROD_ID    = 0x2F;

    // --- Control 0 bits ---
    static const uint8_t TM_M              = 0x01;
    static const uint8_t TM_T              = 0x02;
    static const uint8_t INT_MEAS_DONE_EN  = 0x04;
    static const uint8_t SET_OPERATION     = 0x08;
    static const uint8_t RESET_OPERATION   = 0x10;
    static const uint8_t AUTO_SR_EN        = 0x20;

    // --- Control 1 bits ---
    static const uint8_t BW0       = 0x01;
    static const uint8_t BW1       = 0x02;
    static const uint8_t X_INHIBIT = 0x04;
    static const uint8_t YZ_INHIBIT= 0x18;  // bits 3-4 (Y and Z inhibit)
    static const uint8_t SW_RST    = 0x80;  // bit 7 — datasheet value

    // --- Control 2 bits ---
    static const uint8_t CM_FREQ_0 = 0x01;
    static const uint8_t CM_FREQ_1 = 0x02;
    static const uint8_t CM_FREQ_2 = 0x04;
    static const uint8_t CMM_EN    = 0x08;
    static const uint8_t PRD_SET_0 = 0x10;
    static const uint8_t PRD_SET_1 = 0x20;
    static const uint8_t PRD_SET_2 = 0x40;
    static const uint8_t EN_PRD_SET= 0x80;

    // --- Status bits ---
    static const uint8_t MEAS_M_DONE = 0x01;
    static const uint8_t MEAS_T_DONE = 0x02;

    static const uint8_t PROD_ID_VAL = 0x30;

    // Shadow registers (write-only registers need shadow tracking)
    uint8_t _shadowCtrl0 = 0x00;
    uint8_t _shadowCtrl1 = 0x00;
    uint8_t _shadowCtrl2 = 0x00;
    uint8_t _shadowCtrl3 = 0x00;

    // Calibration state — Gauss domain
    float _bias[3]  = {0.0f, 0.0f, 0.0f};
    float _scale[3] = {1.0f, 1.0f, 1.0f};

    // I2C low-level
    uint8_t readRegister(uint8_t reg);
    void    readRegisters(uint8_t reg, uint8_t* buffer, uint8_t len);
    void    writeRegister(uint8_t reg, uint8_t data);

    // Shadow register helpers
    void shadowSet(uint8_t reg, uint8_t mask, bool write = true);
    void shadowClear(uint8_t reg, uint8_t mask, bool write = true);

public:
    // ================================================================
    // Default config (ES-EKF attitude update, solid rocket)
    //   Edit these constants to retune — begin() applies them.
    //   BW=100 Hz  → 0.4 mG RMS (lowest noise)
    //   CMM=50 Hz  → max ODR allowed at BW=100 (datasheet Max ODR tbl)
    //   PRD_SET=75 → periodic SET every ~1.5 s (bias stability)
    // ================================================================
    static const uint16_t DEFAULT_BW_HZ       = 100;
    static const uint16_t DEFAULT_CMM_HZ      = 50;
    static const uint16_t DEFAULT_PRD_SET_CNT = 75;

    MMC5983MA(TwoWire* wire = &Wire, uint8_t i2cAddr = I2C_ADDR_DEFAULT);

    // Probe + soft reset + applyDefaults()
    bool begin();

    // Re-apply DEFAULT_* config.
    // Use after any temporary change (e.g. calibration boosting CMM rate).
    void applyDefaults();

    // ===== Settings =====
    bool setFilterBandwidth(uint16_t bw);       // 100, 200, 400, 800
    bool setContinuousFrequency(uint16_t freq);  // 0, 1, 10, 20, 50, 100, 200, 1000
    void enableAutoSetReset(bool enable);
    void enableContinuousMode(bool enable);

    // Periodic SET: chip performs SET pulse every N samples for long-term stability.
    // Valid N: 1, 25, 75, 100, 250, 500, 1000, 2000 (PRD_SET[2:0] = 0..7)
    // Call setPeriodicSetCount() BEFORE enablePeriodicSet(true).
    bool setPeriodicSetCount(uint16_t samples);
    void enablePeriodicSet(bool enable);

    void applyConfig(const MMC5983MA_Config& cfg);
    MMC5983MA_Config getConfig();

    // ===== Raw data =====
    // 18-bit raw magnetic field (0 ~ 262143, offset binary: 131072 = 0 Gauss)
    void readRawMag(uint32_t &mx, uint32_t &my, uint32_t &mz);

    // Read field directly in Gauss. Same byte cost as raw (float32 = 4 B)
    // and no precision loss (18-bit fits in 24-bit mantissa).
    // Caller still applies hard-iron / soft-iron calibration.
    void readMag(float &mx, float &my, float &mz);

    // ===== Calibration =====
    // Blocking ~durationMs. User rotates board in figure-8 covering all orientations.
    // Stores hard-iron bias and soft-iron scale internally (in Gauss).
    // Returns true if coverage was sufficient.
    bool calibrate(uint32_t durationMs = 30000);

    // Read fully calibrated Gauss (hard+soft iron + body-frame axis).
    // Body-frame axis: pass-through (edit if MAG is mounted rotated).
    bool readCalibratedMag(float &mx, float &my, float &mz);

    // ===== Interrupt =====
    void enableDataReadyInterrupt(bool enable);
    void clearInterruptFlag();  // Write 1 to Meas_M_Done to release INT pin

    // ===== Polling =====
    bool isDataReady();  // Poll STATUS register for Meas_M_Done

    // ===== Debug =====
    uint8_t debugRead(uint8_t reg) { return readRegister(reg); }
    void    triggerSingleMeasurement();  // Set TM_M; measurement takes ~8 ms

    // Trigger + wait + read, one call. Returns false on timeout.
    bool    readMagSingleShot(uint32_t& mx, uint32_t& my, uint32_t& mz,
                              uint32_t timeoutMs = 20);

    // ===== Utility =====
    bool isConnected();
    void softReset();
    void performSetOperation();
    void performResetOperation();
};

#endif
