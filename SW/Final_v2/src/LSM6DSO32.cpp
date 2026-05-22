#include "LSM6DSO32.h"
#include <math.h>

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
        Serial.printf("LSM6DSO32 error! WHO_AM_I = 0x%02X\n", whoAmI);
        return false;
    }

    writeRegister(REG_CTRL3_C, 0x01);
    delay(10);
    writeRegister(REG_CTRL3_C, 0x44);
    writeRegister(REG_CTRL4_C, 0x04);

    // 416 Hz ODR, +/-32 g, +/-2000 dps.
    writeRegister(REG_CTRL1_XL, 0x64);
    writeRegister(REG_CTRL2_G,  0x6C);

    // Accel LPF2 on, HPCF_XL = 001, cutoff = ODR/10 = 41.6 Hz.
    uint8_t ctrl1 = readRegister(REG_CTRL1_XL);
    writeRegister(REG_CTRL1_XL, ctrl1 | 0x02);
    uint8_t ctrl8 = readRegister(REG_CTRL8_XL);
    ctrl8 &= 0x1F;
    ctrl8 |= (0x01 << 5);
    writeRegister(REG_CTRL8_XL, ctrl8);

    // Gyro LPF1 on, FTYPE = 000, cutoff = 136.6 Hz.
    uint8_t ctrl4 = readRegister(REG_CTRL4_C);
    writeRegister(REG_CTRL4_C, ctrl4 | 0x02);
    uint8_t ctrl6 = readRegister(REG_CTRL6_C);
    ctrl6 &= 0xF8;
    writeRegister(REG_CTRL6_C, ctrl6);

    return true;
}

bool LSM6DSO32::calibrate(uint16_t nSamples) {
    static constexpr int16_t ONE_G_LSB = 1025; // 1 g at +/-32 g, 0.976 mg/LSB.

    int32_t s_gx = 0, s_gy = 0, s_gz = 0, s_ax = 0, s_ay = 0, s_az = 0;
    int64_t ss_gx = 0, ss_gy = 0, ss_gz = 0, ss_ax = 0, ss_ay = 0, ss_az = 0;

    // Collect raw samples while the rocket is vertical and motionless.
    for (uint16_t i = 0; i < nSamples; i++) {
        int16_t gx, gy, gz, ax, ay, az;
        readRawIMU(gx, gy, gz, ax, ay, az);

        s_gx += gx; s_gy += gy; s_gz += gz;
        s_ax += ax; s_ay += ay; s_az += az;

        ss_gx += (int32_t)gx * gx;
        ss_gy += (int32_t)gy * gy;
        ss_gz += (int32_t)gz * gz;
        ss_ax += (int32_t)ax * ax;
        ss_ay += (int32_t)ay * ay;
        ss_az += (int32_t)az * az;

        vTaskDelay(pdMS_TO_TICKS(3));
    }

    float inv_n = 1.0f / (float)nSamples;
    float mean_gx = (float)s_gx * inv_n;
    float mean_gy = (float)s_gy * inv_n;
    float mean_gz = (float)s_gz * inv_n;
    float mean_ax = (float)s_ax * inv_n;
    float mean_ay = (float)s_ay * inv_n;
    float mean_az = (float)s_az * inv_n;

    float std_gx = sqrtf(fmaxf(0.0f, (float)ss_gx * inv_n - mean_gx * mean_gx));
    float std_gy = sqrtf(fmaxf(0.0f, (float)ss_gy * inv_n - mean_gy * mean_gy));
    float std_gz = sqrtf(fmaxf(0.0f, (float)ss_gz * inv_n - mean_gz * mean_gz));
    float std_ax = sqrtf(fmaxf(0.0f, (float)ss_ax * inv_n - mean_ax * mean_ax));
    float std_ay = sqrtf(fmaxf(0.0f, (float)ss_ay * inv_n - mean_ay * mean_ay));
    float std_az = sqrtf(fmaxf(0.0f, (float)ss_az * inv_n - mean_az * mean_az));

    // Publish stats before checks so callers can read them on failure too.
    _lastStats.mean_g[0] = mean_gx; _lastStats.mean_g[1] = mean_gy; _lastStats.mean_g[2] = mean_gz;
    _lastStats.mean_a[0] = mean_ax; _lastStats.mean_a[1] = mean_ay; _lastStats.mean_a[2] = mean_az;
    _lastStats.std_g[0]  = std_gx;  _lastStats.std_g[1]  = std_gy;  _lastStats.std_g[2]  = std_gz;
    _lastStats.std_a[0]  = std_ax;  _lastStats.std_a[1]  = std_ay;  _lastStats.std_a[2]  = std_az;

    if (std_gx > MAX_GYRO_STD_LSB || std_gy > MAX_GYRO_STD_LSB || std_gz > MAX_GYRO_STD_LSB ||
        std_ax > MAX_ACCEL_STD_LSB || std_ay > MAX_ACCEL_STD_LSB || std_az > MAX_ACCEL_STD_LSB) {
        return false;
    }

    _bias_gx = (int16_t)roundf(mean_gx);
    _bias_gy = (int16_t)roundf(mean_gy);
    _bias_gz = (int16_t)roundf(mean_gz);
    _bias_ax = (int16_t)roundf(mean_ax);
    _bias_ay = (int16_t)roundf(mean_ay);
    _bias_az = (int16_t)roundf(mean_az);

    // Keep +1 g on sensor Y, which maps to body X while the rocket is vertical.
    _bias_ay -= ONE_G_LSB;
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

void LSM6DSO32::readRawIMU(int16_t &gx, int16_t &gy, int16_t &gz,
                           int16_t &ax, int16_t &ay, int16_t &az) {
    uint8_t buffer[12];
    readRegisters(REG_OUTX_L_G, buffer, 12);
    gx = (int16_t)((buffer[1]  << 8) | buffer[0]);
    gy = (int16_t)((buffer[3]  << 8) | buffer[2]);
    gz = (int16_t)((buffer[5]  << 8) | buffer[4]);
    ax = (int16_t)((buffer[7]  << 8) | buffer[6]);
    ay = (int16_t)((buffer[9]  << 8) | buffer[8]);
    az = (int16_t)((buffer[11] << 8) | buffer[10]);
}

void LSM6DSO32::readCalibratedIMU(int16_t &gx, int16_t &gy, int16_t &gz,
                                  int16_t &ax, int16_t &ay, int16_t &az) {
    int16_t rgx, rgy, rgz, rax, ray, raz;
    readRawIMU(rgx, rgy, rgz, rax, ray, raz);

    rgx -= _bias_gx; rgy -= _bias_gy; rgz -= _bias_gz;
    rax -= _bias_ax; ray -= _bias_ay; raz -= _bias_az;

    // Body X = Sensor Y (nose direction), Body Y = Sensor X, Body Z = -Sensor Z.
    gx =  rgy;
    gy =  rgx;
    gz = -rgz;

    ax =  ray;
    ay =  rax;
    az = -raz;
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
