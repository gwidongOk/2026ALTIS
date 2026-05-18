#ifndef BMP388_H
#define BMP388_H

#include <Arduino.h>
#include <SPI.h>

class BMP388 {
private:
    uint8_t _csPin;
    SPIClass* _spi;
    SPISettings _spiSettings;

    double par_t1, par_t2, par_t3;
    double par_p1, par_p2, par_p3, par_p4, par_p5, par_p6,
           par_p7, par_p8, par_p9, par_p10, par_p11;

    // Pad pressure stored from calibrate()
    float _pad_p = 101325.0f;

    static const uint8_t REG_CHIP_ID    = 0x00;
    static const uint8_t REG_DATA_0     = 0x04;
    static const uint8_t REG_INT_STATUS = 0x11;
    static const uint8_t REG_INT_CTRL   = 0x19;
    static const uint8_t REG_PWR_CTRL   = 0x1B;
    static const uint8_t REG_OSR        = 0x1C;
    static const uint8_t REG_ODR        = 0x1D;
    static const uint8_t REG_CONFIG     = 0x1F;
    static const uint8_t REG_CALIB_DATA = 0x31;
    static const uint8_t REG_CMD        = 0x7E;

    uint8_t readRegister(uint8_t reg);
    void    readRegisters(uint8_t reg, uint8_t* buffer, uint8_t len);
    void    writeRegister(uint8_t reg, uint8_t data);
    void    readCalibrationData();

public:
    BMP388(uint8_t csPin, SPIClass* spi = &SPI);
    bool begin();
    void setOversampling(uint8_t press_os, uint8_t temp_os);
    void setODR(uint8_t odr);
    void setIIRFilter(uint8_t coef);

    // Compensated absolute pressure (Pa), no pad subtraction
    bool readData(float &pressure);

    // Average pressure at rest, store as _pad_p (called at launch pad)
    // Returns true if samples were stable.
    bool calibrate(uint16_t nSamples = 100);

    // Altitude above pad (m), using stored _pad_p
    bool readAltitude(float &alt);
};

#endif
