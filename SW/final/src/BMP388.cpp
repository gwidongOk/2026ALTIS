#include "BMP388.h"

BMP388::BMP388(uint8_t csPin, SPIClass* spi) {
    _csPin = csPin;
    _spi = spi;
    _spiSettings = SPISettings(5000000, MSBFIRST, SPI_MODE0); 
}

bool BMP388::begin() {
    pinMode(_csPin, OUTPUT);
    digitalWrite(_csPin, HIGH);

    writeRegister(REG_CMD, 0xB6); 
    delay(50);

    uint8_t chipId = readRegister(REG_CHIP_ID);
    if (chipId != 0x50) return false;

    readCalibrationData();

    // 압력 ×8 (osr_p=011), 온도 ×1 (osr_t=000)
    setOversampling(0x03, 0x00);
    // IIR 필터: Off
    setIIRFilter(0x00);
    // ODR: 25Hz (Normal Mode 기준 충분)
    setODR(0x02);               

    writeRegister(REG_INT_CTRL, 0x46);
    delay(10);
    writeRegister(REG_PWR_CTRL, 0x33); 
    return true;
}

void BMP388::setOversampling(uint8_t press_os, uint8_t temp_os) {
    writeRegister(REG_OSR, ((temp_os & 0x07) << 3) | (press_os & 0x07));
}

void BMP388::setODR(uint8_t odr) {
    writeRegister(REG_ODR, odr & 0x1F);
}

void BMP388::setIIRFilter(uint8_t coef) {
    writeRegister(REG_CONFIG, (coef & 0x07) << 1);
}

bool BMP388::readData(float &pressure) {
    readRegister(REG_INT_STATUS); // 인터럽트 핀 해제

    uint8_t data[6];
    readRegisters(REG_DATA_0, data, 6);

    uint32_t uncomp_press = (uint32_t)data[0] | ((uint32_t)data[1] << 8) | ((uint32_t)data[2] << 16);
    uint32_t uncomp_temp = (uint32_t)data[3] | ((uint32_t)data[4] << 8) | ((uint32_t)data[5] << 16);

    double partial_data1 = (double)(uncomp_temp - par_t1);
    double partial_data2 = (double)(partial_data1 * par_t2);
    double t_lin = partial_data2 + (partial_data1 * partial_data1) * par_t3;

    double partial_data1_p = par_p6 * t_lin;
    double partial_data2_p = par_p7 * t_lin * t_lin;
    double partial_data3_p = par_p8 * t_lin * t_lin * t_lin;
    double out1 = par_p5 + partial_data1_p + partial_data2_p + partial_data3_p;

    partial_data1_p = par_p2 * t_lin;
    partial_data2_p = par_p3 * t_lin * t_lin;
    partial_data3_p = par_p4 * t_lin * t_lin * t_lin;
    double out2 = (double)uncomp_press * (par_p1 + partial_data1_p + partial_data2_p + partial_data3_p);

    partial_data1_p = (double)uncomp_press * (double)uncomp_press;
    partial_data2_p = par_p9 + par_p10 * t_lin;
    partial_data3_p = partial_data1_p * partial_data2_p;
    double out3 = partial_data3_p + ((double)uncomp_press * (double)uncomp_press * (double)uncomp_press) * par_p11;

    pressure = (float)(out1 + out2 + out3); 
    return true;
}

void BMP388::calibrate(float &c_p)
{
    float sum_p = 0.0f;
    for (int i = 0; i < 50; i++)
    {
        float d;
        readData(d);
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    for (int i = 0; i < 100; i++)
    {
        float p;
        readData(p);
        sum_p += p;
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    c_p = sum_p / 100.0f;
}

uint8_t BMP388::readRegister(uint8_t reg) {
    uint8_t val;
    _spi->beginTransaction(_spiSettings);
    digitalWrite(_csPin, LOW);
    _spi->transfer(reg | 0x80); 
    _spi->transfer(0x00);       
    val = _spi->transfer(0x00);
    digitalWrite(_csPin, HIGH);
    _spi->endTransaction();
    return val;
}

void BMP388::readRegisters(uint8_t reg, uint8_t* buffer, uint8_t len) {
    _spi->beginTransaction(_spiSettings);
    digitalWrite(_csPin, LOW);
    _spi->transfer(reg | 0x80); 
    _spi->transfer(0x00);       
    for (uint8_t i = 0; i < len; i++) buffer[i] = _spi->transfer(0x00);
    digitalWrite(_csPin, HIGH);
    _spi->endTransaction();
}

void BMP388::writeRegister(uint8_t reg, uint8_t data) {
    _spi->beginTransaction(_spiSettings);
    digitalWrite(_csPin, LOW);
    _spi->transfer(reg & 0x7F); 
    _spi->transfer(data);
    digitalWrite(_csPin, HIGH);
    _spi->endTransaction();
}

void BMP388::readCalibrationData() {
    uint8_t calib[21];
    readRegisters(REG_CALIB_DATA, calib, 21);

    par_t1 = (double)((uint16_t)((calib[1] << 8) | calib[0])) * 256.0;
    par_t2 = (double)((uint16_t)((calib[3] << 8) | calib[2])) / 1073741824.0;
    par_t3 = (double)((int8_t)calib[4]) / 281474976710656.0;
    
    par_p1 = (double)(((int16_t)((calib[6] << 8) | calib[5])) - 16384) / 1048576.0;
    par_p2 = (double)(((int16_t)((calib[8] << 8) | calib[7])) - 16384) / 536870912.0;
    par_p3 = (double)((int8_t)calib[9]) / 4294967296.0;
    par_p4 = (double)((int8_t)calib[10]) / 137438953472.0;
    par_p5 = (double)((uint16_t)((calib[12] << 8) | calib[11])) * 8.0;
    par_p6 = (double)((uint16_t)((calib[14] << 8) | calib[13])) / 64.0;
    par_p7 = (double)((int8_t)calib[15]) / 256.0;
    par_p8 = (double)((int8_t)calib[16]) / 32768.0;
    par_p9 = (double)((int16_t)((calib[18] << 8) | calib[17])) / 281474976710656.0;
    par_p10 = (double)((int8_t)calib[19]) / 281474976710656.0;
    par_p11 = (double)((int8_t)calib[20]) / 36893488147419103232.0;
}