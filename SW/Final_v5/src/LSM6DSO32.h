#ifndef LSM6DSO32_H
#define LSM6DSO32_H

#include <Arduino.h>
#include <SPI.h>

class LSM6DSO32 {
public:
    // Robust calibration thresholds (raw sensor axes, LSB units).
    // "std" in CalibStats is 1.4826 * MAD, a spike-resistant sigma estimate.
    static constexpr float MAX_GYRO_STD_LSB       = 6.0f;
    static constexpr float MAX_ACCEL_STD_LSB      = 20.0f;
    static constexpr float MAX_HORIZONTAL_MEAN_LSB = 180.0f;
    static constexpr float VERTICAL_MEAN_TOL_LSB   = 150.0f;
    static constexpr int16_t ONE_G_LSB              = 1025;

    struct CalibStats {
        float mean_g[3];  // gx, gy, gz (raw sensor axes, LSB)
        float mean_a[3];  // ax, ay, az (raw sensor axes, LSB)
        float std_g[3];
        float std_a[3];
    };

private:
    uint8_t _csPin;
    SPIClass* _spi;
    SPISettings _spiSettings;

    // int16 bias stored at RAW axis (before body-frame remap)
    int16_t _bias_gx = 0, _bias_gy = 0, _bias_gz = 0;
    int16_t _bias_ax = 0, _bias_ay = 0, _bias_az = 0;

    // Stats from the most recent calibrate() call (filled before pass/fail checks)
    CalibStats _lastStats{};
    bool _lastPoseValid = false;

    static const uint8_t REG_INT1_CTRL = 0x0D;
    static const uint8_t REG_INT2_CTRL = 0x0E;
    static const uint8_t REG_WHO_AM_I  = 0x0F;
    static const uint8_t REG_CTRL1_XL  = 0x10;
    static const uint8_t REG_CTRL2_G   = 0x11;
    static const uint8_t REG_CTRL3_C   = 0x12;
    static const uint8_t REG_CTRL4_C   = 0x13;
    static const uint8_t REG_CTRL6_C   = 0x15;
    static const uint8_t REG_CTRL8_XL  = 0x17;
    static const uint8_t REG_OUTX_L_G  = 0x22;
    static const uint8_t REG_OUTX_L_A  = 0x28;

    uint8_t readRegister(uint8_t reg);
    void    readRegisters(uint8_t reg, uint8_t* buffer, uint8_t len);
    void    writeRegister(uint8_t reg, uint8_t data);

public:
    LSM6DSO32(uint8_t csPin, SPIClass* spi = &SPI);

    bool begin();

    void enableAccelDataReadyInterrupt(uint8_t intPin = 1);
    void enableGyroDataReadyInterrupt(uint8_t intPin = 2);

    // 12-byte burst: gyro(0x22~0x27) + accel(0x28~0x2D), no axis/bias
    void readRawIMU(int16_t &gx, int16_t &gy, int16_t &gz,
                    int16_t &ax, int16_t &ay, int16_t &az);

    // Measure one stationary window using a 10% trimmed mean and MAD noise.
    // applyBias=false lets the caller require multiple consecutive windows
    // before committing the averaged bias.
    bool calibrate(uint16_t nSamples = 512, bool applyBias = true);
    void applyCalibration(const CalibStats &stats);

    // Read calibrated: axis-aligned(body frame) + int16 bias subtracted.
    // Body-frame mapping: x = sensor_y, y = sensor_x, z = -sensor_z
    void readCalibratedIMU(int16_t &gx, int16_t &gy, int16_t &gz,
                           int16_t &ax, int16_t &ay, int16_t &az);

    // Mean/std measured during the last calibrate() call (raw sensor axes).
    // Filled even when calibrate() returns false.
    const CalibStats& getLastCalibStats() const { return _lastStats; }
    bool wasLastPoseValid() const { return _lastPoseValid; }
};

#endif
