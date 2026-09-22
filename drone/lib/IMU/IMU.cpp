#include "IMU.h"
#include <Wire.h>
#include "../../src/config.h"

// Registres MPU6050 (datasheet RM-MPU-6000A)
#define MPU6050_ADDR        0x68
#define REG_PWR_MGMT_1      0x6B
#define REG_ACCEL_CONFIG    0x1C
#define REG_GYRO_CONFIG     0x1B
#define REG_ACCEL_XOUT_H    0x3B

// Sensibilites (registres configures en +/-8g et +/-500 deg/s)
static const float LSB_PAR_G     = 4096.0f;   // +/-8g  -> 4096 LSB/g
static const float LSB_PAR_DEGS  = 65.5f;     // +/-500 deg/s -> 65.5 LSB/(deg/s)

IMU::IMU()
    : _offsetGx(0), _offsetGy(0), _offsetGz(0), _premierEchantillon(true) {
    _attitude = {0, 0, 0, 0, 0};
}

bool IMU::begin() {
    Wire.begin();
    Wire.setClock(400000); // I2C rapide (400 kHz) - necessaire pour tenir 250Hz

    // Reveil du capteur (sort du mode sleep)
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(REG_PWR_MGMT_1);
    Wire.write(0x00);
    if (Wire.endTransmission() != 0) return false;

    // Config accelerometre +/-8g
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(REG_ACCEL_CONFIG);
    Wire.write(0x10);
    Wire.endTransmission();

    // Config gyroscope +/-500 deg/s
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(REG_GYRO_CONFIG);
    Wire.write(0x08);
    Wire.endTransmission();

    delay(100);
    return true;
}

void IMU::lireBrut(int16_t& ax, int16_t& ay, int16_t& az,
                     int16_t& gx, int16_t& gy, int16_t& gz) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(REG_ACCEL_XOUT_H);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 14, true);

    ax = (Wire.read() << 8) | Wire.read();
    ay = (Wire.read() << 8) | Wire.read();
    az = (Wire.read() << 8) | Wire.read();
    Wire.read(); Wire.read(); // temperature (ignoree)
    gx = (Wire.read() << 8) | Wire.read();
    gy = (Wire.read() << 8) | Wire.read();
    gz = (Wire.read() << 8) | Wire.read();
}

void IMU::calibrer(uint16_t nEchantillons) {
    long sommeGx = 0, sommeGy = 0, sommeGz = 0;
    int16_t ax, ay, az, gx, gy, gz;

    for (uint16_t i = 0; i < nEchantillons; i++) {
        lireBrut(ax, ay, az, gx, gy, gz);
        sommeGx += gx; sommeGy += gy; sommeGz += gz;
        delay(2);
    }
    _offsetGx = sommeGx / (float)nEchantillons;
    _offsetGy = sommeGy / (float)nEchantillons;
    _offsetGz = sommeGz / (float)nEchantillons;
}

void IMU::mettreAJour(float dtSecondes) {
    int16_t ax, ay, az, gx, gy, gz;
    lireBrut(ax, ay, az, gx, gy, gz);

    // Conversion en unites physiques
    float axg = ax / LSB_PAR_G;
    float ayg = ay / LSB_PAR_G;
    float azg = az / LSB_PAR_G;

    float gxds = (gx - _offsetGx) / LSB_PAR_DEGS;
    float gyds = (gy - _offsetGy) / LSB_PAR_DEGS;
    float gzds = (gz - _offsetGz) / LSB_PAR_DEGS;

    // Angles estimes a partir de l'accelerometre seul (bruites, mais sans derive)
    float roulisAccel  = atan2(ayg, azg) * 180.0f / PI;
    float tangageAccel = atan2(-axg, sqrt(ayg*ayg + azg*azg)) * 180.0f / PI;

    if (_premierEchantillon) {
        _attitude.roulisDeg = roulisAccel;
        _attitude.tangageDeg = tangageAccel;
        _premierEchantillon = false;
    } else {
        // Filtre complementaire : fusion gyro (integration, reactif mais derive)
        // + accelerometre (stable long terme mais bruite)
        _attitude.roulisDeg = FILTRE_ALPHA * (_attitude.roulisDeg + gxds * dtSecondes)
                                + (1.0f - FILTRE_ALPHA) * roulisAccel;
        _attitude.tangageDeg = FILTRE_ALPHA * (_attitude.tangageDeg + gyds * dtSecondes)
                                 + (1.0f - FILTRE_ALPHA) * tangageAccel;
    }

    _attitude.tauxRoulisDegS  = gxds;
    _attitude.tauxTangageDegS = gyds;
    _attitude.tauxLacetDegS   = gzds;
}
