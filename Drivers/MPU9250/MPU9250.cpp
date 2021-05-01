/*
MPU9250.cpp
Brian R Taylor
brian.taylor@bolderflight.com
2017-01-04

Copyright (c) 2016 Bolder Flight Systems

Permission is hereby granted, free of charge, to any person obtaining a copy of this software
and associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or
substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

/* Modified version for use in STM32H7 project by
 * ------------------------------------------
 * Thomas Jespersen, TKJ Electronics
 * Web      :  http://www.tkjelectronics.dk
 * e-mail   :  thomasj@tkjelectronics.dk
 * ------------------------------------------
 */

// Library examples:
// https://github.com/bolderflight/MPU9250/blob/master/src/MPU9250.cpp
// https://github.com/kriswiner/ESP8285/blob/master/MPU9250/MPU9250_MS5637_BasicAHRS2_ESP8266.ino
// https://github.com/kriswiner/MPU9250/blob/master/MPU9250BasicAHRS_t3.ino
// https://github.com/TheChapu/GY-91/blob/master/MPU9250BasicAHRS.ino

#include "MPU9250.h"
#include "Debug.h"
#include "I2C.h" // I2C library
#include "IO.h"
#include "SPI.h" // SPI Library
#include "cmsis_os.h"
#include <math.h>

/* MPU9250 object */
MPU9250::MPU9250(SPI* spi)
    : _interruptPin(0)
    , _interruptSemaphore(0)
    , _accelScale(0)
    , _gyroScale(0)
    , _magScaleX(0)
    , _magScaleY(0)
    , _magScaleZ(0)
{
    _bus = new MPU9250_SPI(spi);
}

MPU9250::MPU9250(I2C* i2c)
    : _interruptPin(0)
    , _interruptSemaphore(0)
    , _accelScale(0)
    , _gyroScale(0)
    , _magScaleX(0)
    , _magScaleY(0)
    , _magScaleZ(0)
{
    _bus = new MPU9250_I2C(i2c);
}

/*MPU9250::MPU9250(PORT sensorPort, uint8_t address) : _accelScale(0), _gyroScale(0), _magScaleX(0), _magScaleY(0),
_magScaleZ(0)
{
    _bus = new COM(sensorPort, address);
}*/

MPU9250::~MPU9250()
{
    if (_interruptSemaphore) {
        _interruptPin->DeregisterInterrupt(); // disable interrupt
        vQueueUnregisterQueue(_interruptSemaphore);
        vSemaphoreDelete(_interruptSemaphore);
    }

    if (_bus)
        delete (_bus);
}

void MPU9250::ConfigureInterrupt(IO* interruptPin)
{
    _interruptPin = interruptPin;

    _interruptSemaphore = xSemaphoreCreateBinary();
    if (_interruptSemaphore == NULL) {
        ERROR("Could not create MPU9250 interrupt semaphore");
        return;
    }
    vQueueAddToRegistry(_interruptSemaphore, "MPU9250 Interrupt");

    enableInt(true);

    _interruptPin->RegisterInterrupt(IO::TRIGGER_RISING, _interruptSemaphore);
}

void MPU9250::ConfigureInterrupt(GPIO_TypeDef* GPIOx, uint32_t GPIO_Pin)
{
    IO* intIO = new IO(GPIOx, GPIO_Pin, IO::PULL_NONE);
    ConfigureInterrupt(intIO);
}

uint32_t MPU9250::WaitForNewData(uint32_t xTicksToWait) // blocking call
{
    if (!_interruptSemaphore)
        return pdFALSE;

    return xSemaphoreTake(_interruptSemaphore, (TickType_t)xTicksToWait);
}

/* starts I2C communication and sets up the MPU-9250 */
int MPU9250::Configure(mpu9250_accel_range accelRange, mpu9250_gyro_range gyroRange)
{
    uint8_t buff[3];
    uint8_t data[7];

    if (!_bus)
        return -1;

    _bus->setBusLowSpeed();

    // select clock source to gyro
    if (!_bus->writeRegister(PWR_MGMNT_1, CLOCK_SEL_PLL)) {
        return -1;
    }

    // enable I2C master mode
    if (!_bus->writeRegister(USER_CTRL, I2C_MST_EN)) {
        return -1;
    }

    // set the I2C bus speed to 400 kHz
    if (!_bus->writeRegister(I2C_MST_CTRL, I2C_MST_CLK)) {
        return -1;
    }

    // set AK8963 to Power Down
    writeAK8963Register(AK8963_CNTL1, AK8963_PWR_DOWN);
    /*if( !writeAK8963Register(AK8963_CNTL1,AK8963_PWR_DOWN) ){
        return -1;
    }*/

    // reset the MPU9250
    _bus->writeRegister(PWR_MGMNT_1, PWR_RESET);

    // wait for MPU-9250 to come back up
    osDelay(1);

    // reset the AK8963
    writeAK8963Register(AK8963_CNTL2, AK8963_RESET);

    // select clock source to gyro
    if (!_bus->writeRegister(PWR_MGMNT_1, CLOCK_SEL_PLL)) {
        return -1;
    }

    // check the WHO AM I byte, expected value is 0x71 (decimal 113)
    if (whoAmI() != 113) {
        return -1;
    }

    // enable accelerometer and gyro
    if (!_bus->writeRegister(PWR_MGMNT_2, SEN_ENABLE)) {
        return -1;
    }

    /*float SelfTestResult[6];
    SelfTest(SelfTestResult);
    _bus->setBusLowSpeed();*/

    /* setup the accel and gyro ranges */
    switch (accelRange) {

        case ACCEL_RANGE_2G:
            // setting the accel range to 2G
            if (!_bus->writeRegister(ACCEL_CONFIG, ACCEL_FS_SEL_2G)) {
                return -1;
            }
            _accelScale = G * 2.0f / 32767.5f; // setting the accel scale to 2G
            break;

        case ACCEL_RANGE_4G:
            // setting the accel range to 4G
            if (!_bus->writeRegister(ACCEL_CONFIG, ACCEL_FS_SEL_4G)) {
                return -1;
            }
            _accelScale = G * 4.0f / 32767.5f; // setting the accel scale to 4G
            break;

        case ACCEL_RANGE_8G:
            // setting the accel range to 8G
            if (!_bus->writeRegister(ACCEL_CONFIG, ACCEL_FS_SEL_8G)) {
                return -1;
            }
            _accelScale = G * 8.0f / 32767.5f; // setting the accel scale to 8G
            break;

        case ACCEL_RANGE_16G:
            // setting the accel range to 16G
            if (!_bus->writeRegister(ACCEL_CONFIG, ACCEL_FS_SEL_16G)) {
                return -1;
            }
            _accelScale = G * 16.0f / 32767.5f; // setting the accel scale to 16G
            break;
    }

    switch (gyroRange) {
        case GYRO_RANGE_250DPS:
            // setting the gyro range to 250DPS
            if (!_bus->writeRegister(GYRO_CONFIG, GYRO_FS_SEL_250DPS)) {
                return -1;
            }
            _gyroScale = 250.0f / 32767.5f * _d2r; // setting the gyro scale to 250DPS
            break;

        case GYRO_RANGE_500DPS:
            // setting the gyro range to 500DPS
            if (!_bus->writeRegister(GYRO_CONFIG, GYRO_FS_SEL_500DPS)) {
                return -1;
            }
            _gyroScale = 500.0f / 32767.5f * _d2r; // setting the gyro scale to 500DPS
            break;

        case GYRO_RANGE_1000DPS:
            // setting the gyro range to 1000DPS
            if (!_bus->writeRegister(GYRO_CONFIG, GYRO_FS_SEL_1000DPS)) {
                return -1;
            }
            _gyroScale = 1000.0f / 32767.5f * _d2r; // setting the gyro scale to 1000DPS
            break;

        case GYRO_RANGE_2000DPS:
            // setting the gyro range to 2000DPS
            if (!_bus->writeRegister(GYRO_CONFIG, GYRO_FS_SEL_2000DPS)) {
                return -1;
            }
            _gyroScale = 2000.0f / 32767.5f * _d2r; // setting the gyro scale to 2000DPS
            break;
    }

    // enable I2C master mode
    if (!_bus->writeRegister(USER_CTRL, I2C_MST_EN)) {
        return -1;
    }

    // set the I2C bus speed to 400 kHz
    if (!_bus->writeRegister(I2C_MST_CTRL, I2C_MST_CLK)) {
        return -1;
    }

    // check AK8963 WHO AM I register, expected value is 0x48 (decimal 72)
    if (whoAmIAK8963() != 72) {
        return -1;
    }

    /* get the magnetometer calibration */

    // set AK8963 to Power Down
    if (!writeAK8963Register(AK8963_CNTL1, AK8963_PWR_DOWN)) {
        return -1;
    }
    osDelay(100); // long wait between AK8963 mode changes

    // set AK8963 to FUSE ROM access
    if (!writeAK8963Register(AK8963_CNTL1, AK8963_FUSE_ROM)) {
        return -1;
    }
    osDelay(100); // long wait between AK8963 mode changes

    // read the AK8963 ASA registers and compute magnetometer scale factors
    readAK8963Registers(AK8963_ASA, sizeof(buff), &buff[0]);
    _magScaleX = ((((float)buff[0]) - 128.0f) / (256.0f) + 1.0f) * 4912.0f / 32760.0f; // micro Tesla
    _magScaleY = ((((float)buff[1]) - 128.0f) / (256.0f) + 1.0f) * 4912.0f / 32760.0f; // micro Tesla
    _magScaleZ = ((((float)buff[2]) - 128.0f) / (256.0f) + 1.0f) * 4912.0f / 32760.0f; // micro Tesla

    // set AK8963 to Power Down
    if (!writeAK8963Register(AK8963_CNTL1, AK8963_PWR_DOWN)) {
        return -1;
    }
    osDelay(100); // long wait between AK8963 mode changes

    // set AK8963 to 16 bit resolution, 100 Hz update rate
    if (!writeAK8963Register(AK8963_CNTL1, AK8963_CNT_MEAS2)) {
        return -1;
    }
    osDelay(100); // long wait between AK8963 mode changes

    // select clock source to gyro
    if (!_bus->writeRegister(PWR_MGMNT_1, CLOCK_SEL_PLL)) {
        return -1;
    }

    // instruct the MPU9250 to get 7 bytes of data from the AK8963 at the sample rate
    readAK8963Registers(AK8963_HXL, sizeof(data), &data[0]);

    _bus->setBusHighSpeed();

    // successful init, return 0
    return 0;
}

/* sets the DLPF and interrupt settings */
int MPU9250::setFilt(mpu9250_dlpf_bandwidth accel_bandwidth, mpu9250_dlpf_bandwidth gyro_bandwidth, uint8_t SRD)
{
    uint8_t data[7];

    _bus->setBusLowSpeed();

    switch (accel_bandwidth) {
        case DLPF_BANDWIDTH_184HZ:
            if (!_bus->writeRegister(ACCEL_CONFIG2, ACCEL_DLPF_184)) { // setting accel bandwidth to 184Hz
                return -1;
            }
            break;

        case DLPF_BANDWIDTH_92HZ:
            if (!_bus->writeRegister(ACCEL_CONFIG2, ACCEL_DLPF_92)) { // setting accel bandwidth to 92Hz
                return -1;
            }
            break;

        case DLPF_BANDWIDTH_41HZ:
            if (!_bus->writeRegister(ACCEL_CONFIG2, ACCEL_DLPF_41)) { // setting accel bandwidth to 41Hz
                return -1;
            }
            break;

        case DLPF_BANDWIDTH_20HZ:
            if (!_bus->writeRegister(ACCEL_CONFIG2, ACCEL_DLPF_20)) { // setting accel bandwidth to 20Hz
                return -1;
            }
            break;

        case DLPF_BANDWIDTH_10HZ:
            if (!_bus->writeRegister(ACCEL_CONFIG2, ACCEL_DLPF_10)) { // setting accel bandwidth to 10Hz
                return -1;
            }
            break;

        case DLPF_BANDWIDTH_5HZ:
            if (!_bus->writeRegister(ACCEL_CONFIG2, ACCEL_DLPF_5)) { // setting accel bandwidth to 5Hz
                return -1;
            }
            break;

        case DLPF_BANDWIDTH_OFF:
            if (!_bus->writeRegister(ACCEL_CONFIG2, ACCEL_DLPF_OFF)) { // setting accel bandwidth to 460Hz
                return -1;
            }
            break;

        case DLPF_BANDWIDTH_250HZ: // incorrect setting
            return -1;
            break;
    }

    switch (gyro_bandwidth) {
        case DLPF_BANDWIDTH_250HZ:
            if (!_bus->writeRegister(CONFIG, GYRO_DLPF_250)) { // setting gyro bandwidth to 184Hz
                return -1;
            }
            break;

        case DLPF_BANDWIDTH_184HZ:
            if (!_bus->writeRegister(CONFIG, GYRO_DLPF_184)) { // setting gyro bandwidth to 184Hz
                return -1;
            }
            break;

        case DLPF_BANDWIDTH_92HZ:
            if (!_bus->writeRegister(CONFIG, GYRO_DLPF_92)) { // setting gyro bandwidth to 92Hz
                return -1;
            }
            break;

        case DLPF_BANDWIDTH_41HZ:
            if (!_bus->writeRegister(CONFIG, GYRO_DLPF_41)) { // setting gyro bandwidth to 41Hz
                return -1;
            }
            break;

        case DLPF_BANDWIDTH_20HZ:
            if (!_bus->writeRegister(CONFIG, GYRO_DLPF_20)) { // setting gyro bandwidth to 20Hz
                return -1;
            }
            break;

        case DLPF_BANDWIDTH_10HZ:
            if (!_bus->writeRegister(CONFIG, GYRO_DLPF_10)) { // setting gyro bandwidth to 10Hz
                return -1;
            }
            break;

        case DLPF_BANDWIDTH_5HZ:
            if (!_bus->writeRegister(CONFIG, GYRO_DLPF_5)) { // setting gyro bandwidth to 5Hz
                return -1;
            }
            break;

        case DLPF_BANDWIDTH_OFF:
            if (!_bus->writeRegister(CONFIG, GYRO_DLPF_OFF)) { // setting gyro bandwidth to 5Hz
                return -1;
            }
            break;
    }

    /* setting the sample rate divider */
    if (!_bus->writeRegister(SMPDIV, SRD)) { // setting the sample rate divider
        return -1;
    }

    if (SRD > 9) {

        // set AK8963 to Power Down
        if (!writeAK8963Register(AK8963_CNTL1, AK8963_PWR_DOWN)) {
            return -1;
        }
        osDelay(100); // long wait between AK8963 mode changes

        // set AK8963 to 16 bit resolution, 8 Hz update rate
        if (!writeAK8963Register(AK8963_CNTL1, AK8963_CNT_MEAS1)) {
            return -1;
        }
        osDelay(100); // long wait between AK8963 mode changes

        // instruct the MPU9250 to get 7 bytes of data from the AK8963 at the sample rate
        readAK8963Registers(AK8963_HXL, sizeof(data), &data[0]);
    }

    /* setting the interrupt */
    /*if( !_bus->writeRegister(INT_PIN_CFG,INT_PULSE_50US) ){ // setup interrupt, 50 us pulse
        return -1;
    }
    if( !_bus->writeRegister(INT_ENABLE,INT_RAW_RDY_EN) ){ // set to data ready
        return -1;
    }*/

    _bus->setBusHighSpeed();

    // successful filter setup, return 0
    return 0;
}

/* enables and disables the interrupt */
int MPU9250::enableInt(bool enable)
{
    _bus->setBusLowSpeed();

    if (enable) {
        /* setting the interrupt */
        if (!_bus->writeRegister(INT_PIN_CFG,
                                 0)) { // setup interrupt, 50 us pulse, active high level, push-pull configuration
            return -1;
        }
        if (!_bus->writeRegister(INT_ENABLE, INT_RAW_RDY_EN)) { // set to data ready
            return -1;
        }
    } else {
        if (!_bus->writeRegister(INT_ENABLE, INT_DISABLE)) { // disable interrupt
            return -1;
        }
    }

    _bus->setBusHighSpeed();

    // successful interrupt setup, return 0
    return 0;
}

/* get accelerometer data given pointers to store the three values, return data as counts */
void MPU9250::getAccelCounts(int16_t* ax, int16_t* ay, int16_t* az)
{
    uint8_t buff[6];
    int16_t axx, ayy, azz;
    //_useSPIHS = true; // use the high speed SPI for data readout

    _bus->readRegisters(ACCEL_OUT, sizeof(buff), &buff[0]); // grab the data from the MPU9250

    axx = (((int16_t)buff[0]) << 8) | buff[1]; // combine into 16 bit values
    ayy = (((int16_t)buff[2]) << 8) | buff[3];
    azz = (((int16_t)buff[4]) << 8) | buff[5];

    *ax = tX[0] * axx + tX[1] * ayy + tX[2] * azz; // transform axes
    *ay = tY[0] * axx + tY[1] * ayy + tY[2] * azz;
    *az = tZ[0] * axx + tZ[1] * ayy + tZ[2] * azz;
}

/* get accelerometer data given pointers to store the three values */
void MPU9250::getAccel(float* ax, float* ay, float* az)
{
    int16_t accel[3];

    getAccelCounts(&accel[0], &accel[1], &accel[2]);

    *ax = ((float)accel[0]) * _accelScale; // typecast and scale to values
    *ay = ((float)accel[1]) * _accelScale;
    *az = ((float)accel[2]) * _accelScale;
}

/* get gyro data given pointers to store the three values, return data as counts */
void MPU9250::getGyroCounts(int16_t* gx, int16_t* gy, int16_t* gz)
{
    uint8_t buff[6];
    int16_t gxx, gyy, gzz;
    //_useSPIHS = true; // use the high speed SPI for data readout

    _bus->readRegisters(GYRO_OUT, sizeof(buff), &buff[0]); // grab the data from the MPU9250

    gxx = (((int16_t)buff[0]) << 8) | buff[1]; // combine into 16 bit values
    gyy = (((int16_t)buff[2]) << 8) | buff[3];
    gzz = (((int16_t)buff[4]) << 8) | buff[5];

    *gx = tX[0] * gxx + tX[1] * gyy + tX[2] * gzz; // transform axes
    *gy = tY[0] * gxx + tY[1] * gyy + tY[2] * gzz;
    *gz = tZ[0] * gxx + tZ[1] * gyy + tZ[2] * gzz;
}

/* get gyro data given pointers to store the three values */
void MPU9250::getGyro(float* gx, float* gy, float* gz)
{
    int16_t gyro[3];

    getGyroCounts(&gyro[0], &gyro[1], &gyro[2]);

    *gx = ((float)gyro[0]) * _gyroScale; // typecast and scale to values
    *gy = ((float)gyro[1]) * _gyroScale;
    *gz = ((float)gyro[2]) * _gyroScale;
}

/* get magnetometer data given pointers to store the three values, return data as counts */
void MPU9250::getMagCounts(int16_t* hx, int16_t* hy, int16_t* hz)
{
    uint8_t buff[7];
    //_useSPIHS = true; // use the high speed SPI for data readout

    // read the magnetometer data off the external sensor buffer
    _bus->readRegisters(EXT_SENS_DATA_00, sizeof(buff), &buff[0]);

    if (buff[6] == 0x10) {                         // check for overflow
        *hx = (((int16_t)buff[1]) << 8) | buff[0]; // combine into 16 bit values
        *hy = (((int16_t)buff[3]) << 8) | buff[2];
        *hz = (((int16_t)buff[5]) << 8) | buff[4];
    } else {
        *hx = 0;
        *hy = 0;
        *hz = 0;
    }
}

/* get magnetometer data given pointers to store the three values */
void MPU9250::getMag(float* hx, float* hy, float* hz)
{
    int16_t mag[3];

    getMagCounts(&mag[0], &mag[1], &mag[2]);

    *hx = ((float)mag[0]) * _magScaleX; // typecast and scale to values
    *hy = ((float)mag[1]) * _magScaleY;
    *hz = ((float)mag[2]) * _magScaleZ;
}

/* get temperature data given pointer to store the value, return data as counts */
void MPU9250::getTempCounts(int16_t* t)
{
    uint8_t buff[2];
    //_useSPIHS = true; // use the high speed SPI for data readout

    _bus->readRegisters(TEMP_OUT, sizeof(buff), &buff[0]); // grab the data from the MPU9250

    *t = (((int16_t)buff[0]) << 8) | buff[1]; // combine into 16 bit value and return
}

/* get temperature data given pointer to store the values */
void MPU9250::getTemp(float* t)
{
    int16_t tempCount;

    getTempCounts(&tempCount);

    *t = ((((float)tempCount) - _tempOffset) / _tempScale) + _tempOffset;
}

/* get accelerometer and gyro data given pointers to store values, return data as counts */
void MPU9250::getMotion6Counts(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz)
{
    uint8_t buff[14];
    int16_t axx, ayy, azz, gxx, gyy, gzz;
    //_useSPIHS = true; // use the high speed SPI for data readout

    _bus->readRegisters(ACCEL_OUT, sizeof(buff), &buff[0]); // grab the data from the MPU9250

    axx = (((int16_t)buff[0]) << 8) | buff[1]; // combine into 16 bit values
    ayy = (((int16_t)buff[2]) << 8) | buff[3];
    azz = (((int16_t)buff[4]) << 8) | buff[5];

    gxx = (((int16_t)buff[8]) << 8) | buff[9];
    gyy = (((int16_t)buff[10]) << 8) | buff[11];
    gzz = (((int16_t)buff[12]) << 8) | buff[13];

    *ax = tX[0] * axx + tX[1] * ayy + tX[2] * azz; // transform axes
    *ay = tY[0] * axx + tY[1] * ayy + tY[2] * azz;
    *az = tZ[0] * axx + tZ[1] * ayy + tZ[2] * azz;

    *gx = tX[0] * gxx + tX[1] * gyy + tX[2] * gzz;
    *gy = tY[0] * gxx + tY[1] * gyy + tY[2] * gzz;
    *gz = tZ[0] * gxx + tZ[1] * gyy + tZ[2] * gzz;
}

/* get accelerometer and gyro data given pointers to store values */
void MPU9250::getMotion6(float* ax, float* ay, float* az, float* gx, float* gy, float* gz)
{
    int16_t accel[3];
    int16_t gyro[3];

    getMotion6Counts(&accel[0], &accel[1], &accel[2], &gyro[0], &gyro[1], &gyro[2]);

    *ax = ((float)accel[0]) * _accelScale; // typecast and scale to values
    *ay = ((float)accel[1]) * _accelScale;
    *az = ((float)accel[2]) * _accelScale;

    *gx = ((float)gyro[0]) * _gyroScale;
    *gy = ((float)gyro[1]) * _gyroScale;
    *gz = ((float)gyro[2]) * _gyroScale;
}

/* get accelerometer, gyro and temperature data given pointers to store values, return data as counts */
void MPU9250::getMotion7Counts(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz, int16_t* t)
{
    uint8_t buff[14];
    int16_t axx, ayy, azz, gxx, gyy, gzz;
    //_useSPIHS = true; // use the high speed SPI for data readout

    _bus->readRegisters(ACCEL_OUT, sizeof(buff), &buff[0]); // grab the data from the MPU9250

    axx = (((int16_t)buff[0]) << 8) | buff[1]; // combine into 16 bit values
    ayy = (((int16_t)buff[2]) << 8) | buff[3];
    azz = (((int16_t)buff[4]) << 8) | buff[5];

    *t = (((int16_t)buff[6]) << 8) | buff[7];

    gxx = (((int16_t)buff[8]) << 8) | buff[9];
    gyy = (((int16_t)buff[10]) << 8) | buff[11];
    gzz = (((int16_t)buff[12]) << 8) | buff[13];

    *ax = tX[0] * axx + tX[1] * ayy + tX[2] * azz; // transform axes
    *ay = tY[0] * axx + tY[1] * ayy + tY[2] * azz;
    *az = tZ[0] * axx + tZ[1] * ayy + tZ[2] * azz;

    *gx = tX[0] * gxx + tX[1] * gyy + tX[2] * gzz;
    *gy = tY[0] * gxx + tY[1] * gyy + tY[2] * gzz;
    *gz = tZ[0] * gxx + tZ[1] * gyy + tZ[2] * gzz;
}

/* get accelerometer, gyro, and temperature data (in SI units) given pointers to store values */
void MPU9250::getMotion7(float* ax, float* ay, float* az, float* gx, float* gy, float* gz, float* t)
{
    int16_t accel[3];
    int16_t gyro[3];
    int16_t tempCount;

    getMotion7Counts(&accel[0], &accel[1], &accel[2], &gyro[0], &gyro[1], &gyro[2], &tempCount);

    *ax = ((float)accel[0]) * _accelScale; // typecast and scale to values
    *ay = ((float)accel[1]) * _accelScale;
    *az = ((float)accel[2]) * _accelScale;

    *gx = ((float)gyro[0]) * _gyroScale;
    *gy = ((float)gyro[1]) * _gyroScale;
    *gz = ((float)gyro[2]) * _gyroScale;

    *t = ((((float)tempCount) - _tempOffset) / _tempScale) + _tempOffset;
}

/* get accelerometer, gyro and magnetometer data given pointers to store values, return data as counts */
void MPU9250::getMotion9Counts(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz,
                               int16_t* hx, int16_t* hy, int16_t* hz)
{
    uint8_t buff[21];
    int16_t axx, ayy, azz, gxx, gyy, gzz;
    //_useSPIHS = true; // use the high speed SPI for data readout

    _bus->readRegisters(ACCEL_OUT, sizeof(buff), &buff[0]); // grab the data from the MPU9250

    axx = (((int16_t)buff[0]) << 8) | buff[1]; // combine into 16 bit values
    ayy = (((int16_t)buff[2]) << 8) | buff[3];
    azz = (((int16_t)buff[4]) << 8) | buff[5];

    gxx = (((int16_t)buff[8]) << 8) | buff[9];
    gyy = (((int16_t)buff[10]) << 8) | buff[11];
    gzz = (((int16_t)buff[12]) << 8) | buff[13];

    *hx = (((int16_t)buff[15]) << 8) | buff[14];
    *hy = (((int16_t)buff[17]) << 8) | buff[16];
    *hz = (((int16_t)buff[19]) << 8) | buff[18];

    *ax = tX[0] * axx + tX[1] * ayy + tX[2] * azz; // transform axes
    *ay = tY[0] * axx + tY[1] * ayy + tY[2] * azz;
    *az = tZ[0] * axx + tZ[1] * ayy + tZ[2] * azz;

    *gx = tX[0] * gxx + tX[1] * gyy + tX[2] * gzz;
    *gy = tY[0] * gxx + tY[1] * gyy + tY[2] * gzz;
    *gz = tZ[0] * gxx + tZ[1] * gyy + tZ[2] * gzz;
}

/* get accelerometer, gyro, and magnetometer data (in SI units) given pointers to store values */
void MPU9250::getMotion9(float* ax, float* ay, float* az, float* gx, float* gy, float* gz, float* hx, float* hy,
                         float* hz)
{
    int16_t accel[3];
    int16_t gyro[3];
    int16_t mag[3];

    getMotion9Counts(&accel[0], &accel[1], &accel[2], &gyro[0], &gyro[1], &gyro[2], &mag[0], &mag[1], &mag[2]);

    *ax = ((float)accel[0]) * _accelScale; // typecast and scale to values
    *ay = ((float)accel[1]) * _accelScale;
    *az = ((float)accel[2]) * _accelScale;

    *gx = ((float)gyro[0]) * _gyroScale;
    *gy = ((float)gyro[1]) * _gyroScale;
    *gz = ((float)gyro[2]) * _gyroScale;

    *hx = ((float)mag[0]) * _magScaleX;
    *hy = ((float)mag[1]) * _magScaleY;
    *hz = ((float)mag[2]) * _magScaleZ;
}

/* get accelerometer, magnetometer, and temperature data (in SI units) given pointers to store values, return data as
 * counts */
void MPU9250::getMotion10Counts(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz,
                                int16_t* hx, int16_t* hy, int16_t* hz, int16_t* t)
{
    uint8_t buff[21];
    int16_t axx, ayy, azz, gxx, gyy, gzz;
    //_useSPIHS = true; // use the high speed SPI for data readout

    _bus->readRegisters(ACCEL_OUT, sizeof(buff), &buff[0]); // grab the data from the MPU9250

    axx = (((int16_t)buff[0]) << 8) | buff[1]; // combine into 16 bit values
    ayy = (((int16_t)buff[2]) << 8) | buff[3];
    azz = (((int16_t)buff[4]) << 8) | buff[5];

    *t = (((int16_t)buff[6]) << 8) | buff[7];

    gxx = (((int16_t)buff[8]) << 8) | buff[9];
    gyy = (((int16_t)buff[10]) << 8) | buff[11];
    gzz = (((int16_t)buff[12]) << 8) | buff[13];

    *hx = (((int16_t)buff[15]) << 8) | buff[14];
    *hy = (((int16_t)buff[17]) << 8) | buff[16];
    *hz = (((int16_t)buff[19]) << 8) | buff[18];

    *ax = tX[0] * axx + tX[1] * ayy + tX[2] * azz; // transform axes
    *ay = tY[0] * axx + tY[1] * ayy + tY[2] * azz;
    *az = tZ[0] * axx + tZ[1] * ayy + tZ[2] * azz;

    *gx = tX[0] * gxx + tX[1] * gyy + tX[2] * gzz;
    *gy = tY[0] * gxx + tY[1] * gyy + tY[2] * gzz;
    *gz = tZ[0] * gxx + tZ[1] * gyy + tZ[2] * gzz;
}

void MPU9250::getMotion10(float* ax, float* ay, float* az, float* gx, float* gy, float* gz, float* hx, float* hy,
                          float* hz, float* t)
{
    int16_t accel[3];
    int16_t gyro[3];
    int16_t mag[3];
    int16_t tempCount;

    getMotion10Counts(&accel[0], &accel[1], &accel[2], &gyro[0], &gyro[1], &gyro[2], &mag[0], &mag[1], &mag[2],
                      &tempCount);

    *ax = ((float)accel[0]) * _accelScale; // typecast and scale to values
    *ay = ((float)accel[1]) * _accelScale;
    *az = ((float)accel[2]) * _accelScale;

    *gx = ((float)gyro[0]) * _gyroScale;
    *gy = ((float)gyro[1]) * _gyroScale;
    *gz = ((float)gyro[2]) * _gyroScale;

    *hx = ((float)mag[0]) * _magScaleX;
    *hy = ((float)mag[1]) * _magScaleY;
    *hz = ((float)mag[2]) * _magScaleZ;

    *t = ((((float)tempCount) - _tempOffset) / _tempScale) + _tempOffset;
}

/* writes a register to the AK8963 given a register address and data */
bool MPU9250::writeAK8963Register(uint8_t subAddress, uint8_t data)
{
    uint8_t count = 1;
    uint8_t buff[1];

    _bus->writeRegister(I2C_SLV0_ADDR, AK8963_I2C_ADDR);     // set slave 0 to the AK8963 and set for write
    _bus->writeRegister(I2C_SLV0_REG, subAddress);           // set the register to the desired AK8963 sub address
    _bus->writeRegister(I2C_SLV0_DO, data);                  // store the data for write
    _bus->writeRegister(I2C_SLV0_CTRL, I2C_SLV0_EN | count); // enable I2C and send 1 byte

    // read the register and confirm
    readAK8963Registers(subAddress, sizeof(buff), &buff[0]);

    if (buff[0] == data) {
        return true;
    } else {
        return false;
    }
}

/* reads registers from the AK8963 */
void MPU9250::readAK8963Registers(uint8_t subAddress, uint8_t count, uint8_t* dest)
{

    _bus->writeRegister(I2C_SLV0_ADDR, AK8963_I2C_ADDR | I2C_READ_FLAG); // set slave 0 to the AK8963 and set for read
    _bus->writeRegister(I2C_SLV0_REG, subAddress);           // set the register to the desired AK8963 sub address
    _bus->writeRegister(I2C_SLV0_CTRL, I2C_SLV0_EN | count); // enable I2C and request the bytes
    HAL_DelayHighRes(1);                                // 100 us wait = takes some time for these registers to fill
    _bus->readRegisters(EXT_SENS_DATA_00, count, dest); // read the bytes off the MPU9250 EXT_SENS_DATA registers
}

/* gets the MPU9250 WHO_AM_I register value, expected to be 0x71 */
uint8_t MPU9250::whoAmI()
{
    uint8_t buff[1];

    // read the WHO AM I register
    _bus->readRegisters(WHO_AM_I, sizeof(buff), &buff[0]);

    // return the register value
    return buff[0];
}

/* gets the AK8963 WHO_AM_I register value, expected to be 0x48 */
uint8_t MPU9250::whoAmIAK8963()
{
    uint8_t buff[1];

    // read the WHO AM I register
    readAK8963Registers(AK8963_WHO_AM_I, sizeof(buff), &buff[0]);

    // return the register value
    return buff[0];
}

// Accelerometer and gyroscope self test; check calibration wrt factory settings
// From https://github.com/kriswiner/ESP8285/blob/master/MPU9250/MPU9250_MS5637_BasicAHRS2_ESP8266.ino
void MPU9250::SelfTest(
  float result[6]) // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
{
    uint8_t rawData[6] = {0, 0, 0, 0, 0, 0};
    uint8_t selfTest[6];
    int32_t gAvg[3] = {0}, aAvg[3] = {0}, aSTAvg[3] = {0}, gSTAvg[3] = {0};
    float   factoryTrim[6];
    uint8_t FS = 0;

    _bus->setBusLowSpeed();

    _bus->writeRegister(SMPDIV, 0x00);          // Set gyro sample rate to 1 kHz
    _bus->writeRegister(CONFIG, 0x02);          // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
    _bus->writeRegister(GYRO_CONFIG, FS << 3);  // Set full scale range for the gyro to 250 dps
    _bus->writeRegister(ACCEL_CONFIG2, 0x02);   // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
    _bus->writeRegister(ACCEL_CONFIG, FS << 3); // Set full scale range for the accelerometer to 2 g

    for (int ii = 0; ii < 200; ii++) { // get average current values of gyro and acclerometer
        _bus->readRegisters(ACCEL_OUT, sizeof(rawData), &rawData[0]); // Read the six raw data registers into data array
        aAvg[0] +=
          (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]); // Turn the MSB and LSB into a signed 16-bit value
        aAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]);
        aAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]);

        _bus->readRegisters(GYRO_OUT, sizeof(rawData),
                            &rawData[0]); // Read the six raw data registers sequentially into data array
        gAvg[0] +=
          (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]); // Turn the MSB and LSB into a signed 16-bit value
        gAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]);
        gAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]);

        osDelay(1);
    }

    for (int ii = 0; ii < 3; ii++) { // Get average of 200 values and store as average current readings
        aAvg[ii] /= 200;
        gAvg[ii] /= 200;
    }

    // Configure the accelerometer for self-test
    _bus->writeRegister(ACCEL_CONFIG,
                        0xE0); // Enable self test on all three axes and set accelerometer range to +/- 2 g
    _bus->writeRegister(GYRO_CONFIG,
                        0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
    osDelay(25);               // osDelay a while to let the device stabilize

    for (int ii = 0; ii < 200; ii++) { // get average self-test values of gyro and acclerometer
        _bus->readRegisters(ACCEL_OUT, sizeof(rawData), &rawData[0]); // Read the six raw data registers into data array
        aSTAvg[0] +=
          (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]); // Turn the MSB and LSB into a signed 16-bit value
        aSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]);
        aSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]);

        _bus->readRegisters(GYRO_OUT, sizeof(rawData),
                            &rawData[0]); // Read the six raw data registers sequentially into data array
        gSTAvg[0] +=
          (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]); // Turn the MSB and LSB into a signed 16-bit value
        gSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]);
        gSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]);

        osDelay(1);
    }

    for (int ii = 0; ii < 3; ii++) { // Get average of 200 values and store as average self-test readings
        aSTAvg[ii] /= 200;
        gSTAvg[ii] /= 200;
    }

    // Configure the gyro and accelerometer for normal operation
    _bus->writeRegister(ACCEL_CONFIG, 0x00);
    _bus->writeRegister(GYRO_CONFIG, 0x00);
    osDelay(25); // osDelay a while to let the device stabilize

    // Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
    _bus->readRegisters(SELF_TEST_X_ACCEL, 1, &selfTest[0]); // X-axis accel self-test results
    _bus->readRegisters(SELF_TEST_Y_ACCEL, 1, &selfTest[1]); // Y-axis accel self-test results
    _bus->readRegisters(SELF_TEST_Z_ACCEL, 1, &selfTest[2]); // Z-axis accel self-test results
    _bus->readRegisters(SELF_TEST_X_GYRO, 1, &selfTest[3]);  // X-axis gyro self-test results
    _bus->readRegisters(SELF_TEST_Y_GYRO, 1, &selfTest[4]);  // Y-axis gyro self-test results
    _bus->readRegisters(SELF_TEST_Z_GYRO, 1, &selfTest[5]);  // Z-axis gyro self-test results

    // Retrieve factory self-test value from self-test code reads
    factoryTrim[0] =
      (float)(2620 / 1 << FS) * (powf(1.01, ((float)selfTest[0] - 1.0))); // FT[Xa] factory trim calculation
    factoryTrim[1] =
      (float)(2620 / 1 << FS) * (powf(1.01, ((float)selfTest[1] - 1.0))); // FT[Ya] factory trim calculation
    factoryTrim[2] =
      (float)(2620 / 1 << FS) * (powf(1.01, ((float)selfTest[2] - 1.0))); // FT[Za] factory trim calculation
    factoryTrim[3] =
      (float)(2620 / 1 << FS) * (powf(1.01, ((float)selfTest[3] - 1.0))); // FT[Xg] factory trim calculation
    factoryTrim[4] =
      (float)(2620 / 1 << FS) * (powf(1.01, ((float)selfTest[4] - 1.0))); // FT[Yg] factory trim calculation
    factoryTrim[5] =
      (float)(2620 / 1 << FS) * (powf(1.01, ((float)selfTest[5] - 1.0))); // FT[Zg] factory trim calculation

    // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
    // To get percent, must multiply by 100
    for (int i = 0; i < 3; i++) {
        result[i] = 100.0 * ((float)(aSTAvg[i] - aAvg[i])) / factoryTrim[i] - 100.; // Report percent differences
        result[i + 3] =
          100.0 * ((float)(gSTAvg[i] - gAvg[i])) / factoryTrim[i + 3] - 100.; // Report percent differences
    }

    _bus->setBusHighSpeed();
}

void MPU9250::CalibrateMagnetometer(float* dest1, float* dest2)
{
    uint16_t ii = 0, sample_count = 0;
    int32_t  mag_bias[3] = {0, 0, 0}, mag_scale[3] = {0, 0, 0};
    int16_t  mag_max[3] = {-32767, -32767, -32767}, mag_min[3] = {32767, 32767, 32767}, mag_temp[3] = {0, 0, 0};

    _bus->setBusLowSpeed();

    DEBUG("Mag Calibration: Wave device in a figure eight until done!");
    osDelay(4000);

    // shoot for ~fifteen seconds of mag data
    // sample_count = 128;  // at 8 Hz ODR, new mag data is available every 125 ms
    sample_count = 1500; // at 100 Hz ODR, new mag data is available every 10 ms
    for (ii = 0; ii < sample_count; ii++) {
        getMagCounts(&mag_temp[0], &mag_temp[1], &mag_temp[2]); // Read the mag data
        for (int jj = 0; jj < 3; jj++) {
            if (mag_temp[jj] > mag_max[jj])
                mag_max[jj] = mag_temp[jj];
            if (mag_temp[jj] < mag_min[jj])
                mag_min[jj] = mag_temp[jj];
        }
        // osDelay(135);  // at 8 Hz ODR, new mag data is available every 125 ms
        osDelay(12); // at 100 Hz ODR, new mag data is available every 10 ms
    }

    //    Serial.println("mag x min/max:"); Serial.println(mag_max[0]); Serial.println(mag_min[0]);
    //    Serial.println("mag y min/max:"); Serial.println(mag_max[1]); Serial.println(mag_min[1]);
    //    Serial.println("mag z min/max:"); Serial.println(mag_max[2]); Serial.println(mag_min[2]);

    // Get hard iron correction
    mag_bias[0] = (mag_max[0] + mag_min[0]) / 2; // get average x mag bias in counts
    mag_bias[1] = (mag_max[1] + mag_min[1]) / 2; // get average y mag bias in counts
    mag_bias[2] = (mag_max[2] + mag_min[2]) / 2; // get average z mag bias in counts

    dest1[0] = (float)mag_bias[0] * _magScaleX; // save mag biases in G for main program
    dest1[1] = (float)mag_bias[1] * _magScaleY;
    dest1[2] = (float)mag_bias[2] * _magScaleZ;

    // Get soft iron correction estimate
    mag_scale[0] = (mag_max[0] - mag_min[0]) / 2; // get average x axis max chord length in counts
    mag_scale[1] = (mag_max[1] - mag_min[1]) / 2; // get average y axis max chord length in counts
    mag_scale[2] = (mag_max[2] - mag_min[2]) / 2; // get average z axis max chord length in counts

    float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
    avg_rad /= 3.0;

    dest2[0] = avg_rad / ((float)mag_scale[0]);
    dest2[1] = avg_rad / ((float)mag_scale[1]);
    dest2[2] = avg_rad / ((float)mag_scale[2]);

    _bus->setBusHighSpeed();

    DEBUG("Mag Calibration done!");
}

void MPU9250::Get(Measurement_t& measurement)
{
    getMotion9(&measurement.Accelerometer[0], &measurement.Accelerometer[1], &measurement.Accelerometer[2],
               &measurement.Gyroscope[0], &measurement.Gyroscope[1], &measurement.Gyroscope[2],
               &measurement.Magnetometer[0], &measurement.Magnetometer[1], &measurement.Magnetometer[2]);
}
