#include "LSM6DSO32.h"

LSM6DSO32::LSM6DSO32(uint8_t csPin, SPIClass* spi) {
    _csPin = csPin;
    _spi = spi;
    _spiSettings = SPISettings(5000000, MSBFIRST, SPI_MODE3); 
}

bool LSM6DSO32::begin() {
    pinMode(_csPin, OUTPUT);
    digitalWrite(_csPin, HIGH);

    uint8_t whoAmI = readRegister(REG_WHO_AM_I);
    if (whoAmI != 0x6C) {
        Serial.printf("LSM6DSO32 에러! WHO_AM_I = 0x%02X\n", whoAmI);
        return false;
    }

    writeRegister(REG_CTRL3_C, 0x01);
    delay(10);
    writeRegister(REG_CTRL3_C, 0x44);
    writeRegister(REG_CTRL4_C, 0x04);

    // 416Hz ODR, ±32g, ±2000dps
    writeRegister(REG_CTRL1_XL, 0x64); 
    writeRegister(REG_CTRL2_G, 0x6C);

    // LPF 세팅
    uint8_t ctrl1 = readRegister(REG_CTRL1_XL);
    writeRegister(REG_CTRL1_XL, ctrl1 | 0x02); 
    uint8_t ctrl8 = readRegister(REG_CTRL8_XL);
    ctrl8 &= 0x1F;       
    ctrl8 |= (0x02 << 5); 
    writeRegister(REG_CTRL8_XL, ctrl8);

    uint8_t ctrl4 = readRegister(REG_CTRL4_C);
    writeRegister(REG_CTRL4_C, ctrl4 | 0x02); 
    uint8_t ctrl6 = readRegister(REG_CTRL6_C);
    ctrl6 &= 0xF8;       
    ctrl6 |= 0x01;       
    writeRegister(REG_CTRL6_C, ctrl6);

    return true;
}

void LSM6DSO32::enableAccelDataReadyInterrupt(uint8_t intPin) {
    uint8_t reg = (intPin == 1) ? REG_INT1_CTRL : REG_INT2_CTRL;
    writeRegister(reg, readRegister(reg) | 0x01); 
}

void LSM6DSO32::enableGyroDataReadyInterrupt(uint8_t intPin) {
    uint8_t reg = (intPin == 1) ? REG_INT1_CTRL : REG_INT2_CTRL;
    writeRegister(reg, readRegister(reg) | 0x02); 
}

void LSM6DSO32::readRawAccel(int16_t &ax, int16_t &ay, int16_t &az) {
    uint8_t buffer[6];
    readRegisters(REG_OUTX_L_A, buffer, 6);
    ax = (int16_t)((buffer[1] << 8) | buffer[0]);
    ay = (int16_t)((buffer[3] << 8) | buffer[2]);
    az = (int16_t)((buffer[5] << 8) | buffer[4]);
}

void LSM6DSO32::readRawGyro(int16_t &gx, int16_t &gy, int16_t &gz) {
    uint8_t buffer[6];
    readRegisters(REG_OUTX_L_G, buffer, 6);
    gx = (int16_t)((buffer[1] << 8) | buffer[0]);
    gy = (int16_t)((buffer[3] << 8) | buffer[2]);
    gz = (int16_t)((buffer[5] << 8) | buffer[4]);
}

uint8_t LSM6DSO32::readRegister(uint8_t reg) {
    uint8_t value;
    _spi->beginTransaction(_spiSettings);
    digitalWrite(_csPin, LOW);
    _spi->transfer(reg | 0x80); 
    value = _spi->transfer(0x00);
    digitalWrite(_csPin, HIGH);
    _spi->endTransaction();
    return value;
}

void LSM6DSO32::readRegisters(uint8_t reg, uint8_t* buffer, uint8_t len) {
    _spi->beginTransaction(_spiSettings);
    digitalWrite(_csPin, LOW);
    _spi->transfer(reg | 0x80); 
    for (uint8_t i = 0; i < len; i++) buffer[i] = _spi->transfer(0x00);
    digitalWrite(_csPin, HIGH);
    _spi->endTransaction();
}

void LSM6DSO32::writeRegister(uint8_t reg, uint8_t data) {
    _spi->beginTransaction(_spiSettings);
    digitalWrite(_csPin, LOW);
    _spi->transfer(reg & 0x7F); 
    _spi->transfer(data);
    digitalWrite(_csPin, HIGH);
    _spi->endTransaction();
}