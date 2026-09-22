/**
 * Actionneurs.h - Pilotage des moteurs CC via pont en H (L298N)
 */
#ifndef ACTIONNEURS_H
#define ACTIONNEURS_H

#include <Arduino.h>

class MoteurCC {
public:
    MoteurCC(uint8_t pinIn1, uint8_t pinIn2, uint8_t pinPwm);
    void begin();
    // vitesse dans [-255, 255] : signe = sens de rotation
    void commander(int16_t vitesse);
    void arreter();

private:
    uint8_t _in1, _in2, _pwm;
};

class Chassis {
public:
    Chassis(MoteurCC& moteurGauche, MoteurCC& moteurDroit);
    void begin();
    // v : vitesse lineaire de base [-255,255], correction : différentiel ajouté/soustrait
    void deplacer(int16_t v, int16_t correction);
    void stop();
    void pivoterSurPlace(int16_t vitesse); // >0 = pivote a droite

private:
    MoteurCC& _moteurG;
    MoteurCC& _moteurD;
};

#endif // ACTIONNEURS_H
