#include "LSM6DSO32.h"
#include <algorithm>
#include <math.h>

namespace {
constexpr uint16_t MAX_CALIB_WINDOW_SAMPLES = 512;

struct CalibSample {
    int16_t gx, gy, gz;
    int16_t ax, ay, az;
};

// Static scratch avoids placing about 9 kB on the Arduino loop-task stack.
static CalibSample calibSamples[MAX_CALIB_WINDOW_SAMPLES];
static int16_t     sortedAxis[MAX_CALIB_WINDOW_SAMPLES];
static float       sortedDeviation[MAX_CALIB_WINDOW_SAMPLES];

int16_t getAxisValue(const CalibSample &sample, uint8_t axis) {
    switch (axis) {
        case 0: return sample.gx;
        case 1: return sample.gy;
        case 2: return sample.gz;
        case 3: return sample.ax;
        case 4: return sample.ay;
        default: return sample.az;
    }
}

void calculateRobustStats(uint16_t count, uint8_t axis,
                          float &trimmedMean, float &robustSigma) {
    for (uint16_t i = 0; i < count; i++) {
        sortedAxis[i] = getAxisValue(calibSamples[i], axis);
    }
    std::sort(sortedAxis, sortedAxis + count);

    // Remove the highest and lowest 10% before calculating the bias mean.
    uint16_t trim = count / 10;
    uint16_t first = trim;
    uint16_t last = count - trim;
    double sum = 0.0;
    for (uint16_t i = first; i < last; i++) sum += sortedAxis[i];
    trimmedMean = (float)(sum / (double)(last - first));

    float median;
    if ((count & 1U) == 0) {
        median = 0.5f * ((float)sortedAxis[count / 2 - 1] +
                         (float)sortedAxis[count / 2]);
    } else {
        median = (float)sortedAxis[count / 2];
    }

    for (uint16_t i = 0; i < count; i++) {
        sortedDeviation[i] =
            fabsf((float)getAxisValue(calibSamples[i], axis) - median);
    }
    std::sort(sortedDeviation, sortedDeviation + count);

    float mad;
    if ((count & 1U) == 0) {
        mad = 0.5f * (sortedDeviation[count / 2 - 1] +
                      sortedDeviation[count / 2]);
    } else {
        mad = sortedDeviation[count / 2];
    }

    // Gaussian-equivalent sigma estimate, robust against isolated shocks.
    robustSigma = 1.4826f * mad;
}
}

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

bool LSM6DSO32::calibrate(uint16_t nSamples, bool applyBias) {
    if (nSamples < 32) return false;
    if (nSamples > MAX_CALIB_WINDOW_SAMPLES) {
        nSamples = MAX_CALIB_WINDOW_SAMPLES;
    }

    // Collect raw samples while the rocket is vertical and motionless.
    for (uint16_t i = 0; i < nSamples; i++) {
        readRawIMU(calibSamples[i].gx, calibSamples[i].gy,
                   calibSamples[i].gz, calibSamples[i].ax,
                   calibSamples[i].ay, calibSamples[i].az);
        vTaskDelay(pdMS_TO_TICKS(3));
    }

    for (uint8_t axis = 0; axis < 6; axis++) {
        float mean, sigma;
        calculateRobustStats(nSamples, axis, mean, sigma);
        if (axis < 3) {
            _lastStats.mean_g[axis] = mean;
            _lastStats.std_g[axis] = sigma;
        } else {
            _lastStats.mean_a[axis - 3] = mean;
            _lastStats.std_a[axis - 3] = sigma;
        }
    }

    bool noiseValid =
        _lastStats.std_g[0] <= MAX_GYRO_STD_LSB &&
        _lastStats.std_g[1] <= MAX_GYRO_STD_LSB &&
        _lastStats.std_g[2] <= MAX_GYRO_STD_LSB &&
        _lastStats.std_a[0] <= MAX_ACCEL_STD_LSB &&
        _lastStats.std_a[1] <= MAX_ACCEL_STD_LSB &&
        _lastStats.std_a[2] <= MAX_ACCEL_STD_LSB;

    // Calibration is only valid in the known vertical pose:
    // raw sensor Y carries +1 g and X/Z should be close to zero.
    _lastPoseValid =
        fabsf(_lastStats.mean_a[0]) <= MAX_HORIZONTAL_MEAN_LSB &&
        fabsf(_lastStats.mean_a[2]) <= MAX_HORIZONTAL_MEAN_LSB &&
        fabsf(_lastStats.mean_a[1] - (float)ONE_G_LSB) <=
            VERTICAL_MEAN_TOL_LSB;

    bool valid = noiseValid && _lastPoseValid;
    if (valid && applyBias) applyCalibration(_lastStats);
    return valid;
}

void LSM6DSO32::applyCalibration(const CalibStats &stats) {
    _bias_gx = (int16_t)roundf(stats.mean_g[0]);
    _bias_gy = (int16_t)roundf(stats.mean_g[1]);
    _bias_gz = (int16_t)roundf(stats.mean_g[2]);
    _bias_ax = (int16_t)roundf(stats.mean_a[0]);
    _bias_ay = (int16_t)roundf(stats.mean_a[1]);
    _bias_az = (int16_t)roundf(stats.mean_a[2]);

    // Keep +1 g on sensor Y, which maps to body X while the rocket is vertical.
    _bias_ay -= ONE_G_LSB;
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
