/**
 * IMU.h - Lecture MPU6050 (accelerometre + gyroscope 6 axes) et fusion
 * par filtre complementaire pour estimer les angles de roulis/tangage.
 *
 *   angle_k = alpha*(angle_(k-1) + gyro*Ts) + (1-alpha)*angle_accel
 *
 * Ce filtre est plus economique qu'un filtre de Kalman et suffisant
 * pour la stabilisation d'un mini-drone (cf. rapport section 2.6).
 */
#ifndef IMU_H
#define IMU_H

#include <Arduino.h>

struct AttitudeIMU {
    float roulisDeg;
    float tangageDeg;
    float tauxRoulisDegS;   // vitesse angulaire (sortie brute gyro, filtree passe-bas)
    float tauxTangageDegS;
    float tauxLacetDegS;
};

class IMU {
public:
    IMU();
    bool begin();                 // initialise le MPU6050 (I2C), retourne false si non detecte
    void calibrer(uint16_t nEchantillons = 500); // calcul des offsets gyro au repos
    void mettreAJour(float dtSecondes);          // a appeler a chaque cycle de boucle
    AttitudeIMU obtenirAttitude() const { return _attitude; }

private:
    void lireBrut(int16_t& ax, int16_t& ay, int16_t& az,
                   int16_t& gx, int16_t& gy, int16_t& gz);

    float _offsetGx, _offsetGy, _offsetGz;
    AttitudeIMU _attitude;
    bool _premierEchantillon;
};

#endif // IMU_H
