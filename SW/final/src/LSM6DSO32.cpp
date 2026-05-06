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

    // ---- 가속도 LPF2 세팅 ----
    // LPF2_XL_EN = 1 (CTRL1_XL bit1)
    uint8_t ctrl1 = readRegister(REG_CTRL1_XL);
    writeRegister(REG_CTRL1_XL, ctrl1 | 0x02);

    // HPCF_XL = 001 → cutoff = ODR/10 = 41.6 Hz (CTRL8_XL[7:5])
    uint8_t ctrl8 = readRegister(REG_CTRL8_XL);
    ctrl8 &= 0x1F;                // bits[7:5] 클리어
    ctrl8 |= (0x01 << 5);         // HPCF_XL = 001
    writeRegister(REG_CTRL8_XL, ctrl8);

    // ---- 자이로 LPF1 세팅 ----
    // LPF1_SEL_G = 1 → LPF1 출력 선택 (CTRL4_C bit1)
    uint8_t ctrl4 = readRegister(REG_CTRL4_C);
    writeRegister(REG_CTRL4_C, ctrl4 | 0x02);

    // FTYPE = 000 → cutoff = 136.6 Hz (CTRL6_C[2:0])
    uint8_t ctrl6 = readRegister(REG_CTRL6_C);
    ctrl6 &= 0xF8;                // bits[2:0] 클리어 → FTYPE = 000
    writeRegister(REG_CTRL6_C, ctrl6);

    return true;
}

void LSM6DSO32::calibrate(float &c_gx, float &c_gy, float &c_gz, 
                          float &c_ax, float &c_ay, float &c_az) {
    
  int32_t sum_gx = 0, sum_gy = 0, sum_gz = 0;
  int32_t sum_ax = 0, sum_ay = 0, sum_az = 0;

  for (int i = 0; i < 100; i++) {
    int16_t gx, gy, gz;
    int16_t ax, ay, az;
    
    readRawIMU(gx, gy, gz, ax, ay, az);
    
    sum_gx += gx; sum_gy += gy; sum_gz += gz;
    sum_ax += ax; sum_ay += ay; sum_az += az;

    vTaskDelay(pdMS_TO_TICKS(3));  // 416Hz = 2.4ms, 3ms로 여유
  }

  c_gx = sum_gx / 100.0f; 
  c_gy = sum_gy / 100.0f; 
  c_gz = sum_gz / 100.0f;
  c_ax = sum_ax / 100.0f; 
  c_ay = sum_ay / 100.0f; 
  c_az = sum_az / 100.0f;
}


void LSM6DSO32::enableAccelDataReadyInterrupt(uint8_t intPin) {
    uint8_t reg = (intPin == 1) ? REG_INT1_CTRL : REG_INT2_CTRL;
    writeRegister(reg, readRegister(reg) | 0x01); 
}

void LSM6DSO32::enableGyroDataReadyInterrupt(uint8_t intPin) {
    uint8_t reg = (intPin == 1) ? REG_INT1_CTRL : REG_INT2_CTRL;
    writeRegister(reg, readRegister(reg) | 0x02); 
}

void LSM6DSO32::readRawIMU(int16_t &gx, int16_t &gy, int16_t &gz,
                           int16_t &ax, int16_t &ay, int16_t &az) {
    uint8_t buffer[12];
    readRegisters(REG_OUTX_L_G, buffer, 12);  // 0x22~0x2D 연속 읽기
    gx = (int16_t)((buffer[1]  << 8) | buffer[0]);
    gy = (int16_t)((buffer[3]  << 8) | buffer[2]);
    gz = (int16_t)((buffer[5]  << 8) | buffer[4]);
    ax = (int16_t)((buffer[7]  << 8) | buffer[6]);
    ay = (int16_t)((buffer[9]  << 8) | buffer[8]);
    az = (int16_t)((buffer[11] << 8) | buffer[10]);
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