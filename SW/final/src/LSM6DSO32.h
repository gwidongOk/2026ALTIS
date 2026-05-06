#ifndef LSM6DSO32_H
#define LSM6DSO32_H

#include <Arduino.h>
#include <SPI.h>

class LSM6DSO32 {
private:
    uint8_t _csPin;
    SPIClass* _spi;
    SPISettings _spiSettings;

    // 주요 레지스터 주소
    static const uint8_t REG_INT1_CTRL = 0x0D;
    static const uint8_t REG_INT2_CTRL = 0x0E;
    static const uint8_t REG_WHO_AM_I = 0x0F;
    static const uint8_t REG_CTRL1_XL = 0x10;
    static const uint8_t REG_CTRL2_G = 0x11;
    static const uint8_t REG_CTRL3_C = 0x12;
    static const uint8_t REG_CTRL4_C = 0x13;
    static const uint8_t REG_OUTX_L_G = 0x22;
    static const uint8_t REG_OUTX_L_A = 0x28;
    static const uint8_t REG_CTRL6_C = 0x15;
    static const uint8_t REG_CTRL8_XL = 0x17;

    uint8_t readRegister(uint8_t reg);
    void readRegisters(uint8_t reg, uint8_t* buffer, uint8_t len);
    void writeRegister(uint8_t reg, uint8_t data);

public:
    LSM6DSO32(uint8_t csPin, SPIClass* spi = &SPI);

    bool begin();

    void enableAccelDataReadyInterrupt(uint8_t intPin = 1);
    void enableGyroDataReadyInterrupt(uint8_t intPin = 2);

    // Gyro(0x22) + Accel(0x28) 12바이트 버스트 리드
    void readRawIMU(int16_t &gx, int16_t &gy, int16_t &gz,
                    int16_t &ax, int16_t &ay, int16_t &az);

    void calibrate(float &gx, float &gy, float &gz,float &ax, float &ay, float &az);
};

#endif