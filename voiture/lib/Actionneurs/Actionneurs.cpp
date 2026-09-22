#include "Actionneurs.h"
#include "../../src/config.h"

// =======================================================================
//  MoteurCC
// =======================================================================
MoteurCC::MoteurCC(uint8_t pinIn1, uint8_t pinIn2, uint8_t pinPwm)
    : _in1(pinIn1), _in2(pinIn2), _pwm(pinPwm) {}

void MoteurCC::begin() {
    pinMode(_in1, OUTPUT);
    pinMode(_in2, OUTPUT);
    pinMode(_pwm, OUTPUT);
    arreter();
}

void MoteurCC::commander(int16_t vitesse) {
    vitesse = constrain(vitesse, -PWM_MAX, PWM_MAX);

    if (vitesse > 0) {
        digitalWrite(_in1, HIGH);
        digitalWrite(_in2, LOW);
    } else if (vitesse < 0) {
        digitalWrite(_in1, LOW);
        digitalWrite(_in2, HIGH);
    } else {
        digitalWrite(_in1, LOW);
        digitalWrite(_in2, LOW);
    }
    analogWrite(_pwm, abs(vitesse));
}

void MoteurCC::arreter() {
    digitalWrite(_in1, LOW);
    digitalWrite(_in2, LOW);
    analogWrite(_pwm, 0);
}

// =======================================================================
//  Chassis (2 roues motrices)
// =======================================================================
Chassis::Chassis(MoteurCC& moteurGauche, MoteurCC& moteurDroit)
    : _moteurG(moteurGauche), _moteurD(moteurDroit) {}

void Chassis::begin() {
    _moteurG.begin();
    _moteurD.begin();
}

void Chassis::deplacer(int16_t v, int16_t correction) {
    // correction > 0 -> on ralentit la roue droite / accelere la gauche (tourne a droite)
    int16_t vG = v + correction;
    int16_t vD = v - correction;
    _moteurG.commander(vG);
    _moteurD.commander(vD);
}

void Chassis::stop() {
    _moteurG.arreter();
    _moteurD.arreter();
}

void Chassis::pivoterSurPlace(int16_t vitesse) {
    _moteurG.commander(vitesse);
    _moteurD.commander(-vitesse);
}
