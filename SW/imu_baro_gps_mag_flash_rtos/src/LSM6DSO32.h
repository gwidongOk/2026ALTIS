#ifndef LSM6DSO32_H
#define LSM6DSO32_H

#include <Arduino.h>
#include <SPI.h>

class LSM6DSO32 {
private:
    uint8_t _csPin;
    SPIClass* _spi;
    SPISettings _spiSettings;

    // int16 bias stored at RAW axis (before body-frame remap)
    int16_t _bias_gx = 0, _bias_gy = 0, _bias_gz = 0;
    int16_t _bias_ax = 0, _bias_ay = 0, _bias_az = 0;

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

    // Calibrate nSamples at rest; stores int16 RAW bias internally.
    // Returns true if samples were stable (low variance).
    bool calibrate(uint16_t nSamples = 100);

    // Read calibrated: axis-aligned(body frame) + int16 bias subtracted.
    // Body-frame mapping: x = sensor_y, y = sensor_x, z = -sensor_z
    void readCalibratedIMU(int16_t &gx, int16_t &gy, int16_t &gz,
                           int16_t &ax, int16_t &ay, int16_t &az);
};

#endif
