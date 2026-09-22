#include "PIDController.h"

PIDController::PIDController(float kp, float ki, float kd, float sortieMin, float sortieMax)
    : _kp(kp), _ki(ki), _kd(kd), _sortieMin(sortieMin), _sortieMax(sortieMax),
      _integrale(0), _erreurPrecedente(0), _premierAppel(true) {}

void PIDController::reinitialiser() {
    _integrale = 0;
    _erreurPrecedente = 0;
    _premierAppel = true;
}

void PIDController::reglerGains(float kp, float ki, float kd) {
    _kp = kp; _ki = ki; _kd = kd;
}

float PIDController::calculer(float consigne, float mesure, float dt) {
    if (dt <= 0) return 0;

    float erreur = consigne - mesure;

    // Terme derive (nul au premier appel pour eviter un "coup" de derivee)
    float derivee = 0;
    if (!_premierAppel) {
        derivee = (erreur - _erreurPrecedente) / dt;
    }
    _premierAppel = false;

    // Terme integral avec anti-windup par clamping conditionnel
    float integraleEssai = _integrale + erreur * dt;
    float sortieEssai = _kp * erreur + _ki * integraleEssai + _kd * derivee;

    if (sortieEssai > _sortieMax || sortieEssai < _sortieMin) {
        // Saturation : on n'accumule pas davantage l'integrale (anti-windup)
    } else {
        _integrale = integraleEssai;
    }

    float sortie = _kp * erreur + _ki * _integrale + _kd * derivee;
    sortie = constrain(sortie, _sortieMin, _sortieMax);

    _erreurPrecedente = erreur;
    return sortie;
}
