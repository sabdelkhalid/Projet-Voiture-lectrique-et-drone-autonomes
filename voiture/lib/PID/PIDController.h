/**
 * PIDController.h - Correcteur PID générique discret, avec anti-windup
 * par écrêtage de l'intégrateur (clamping) et limitation de sortie.
 *
 * Utilisé pour : régulation de vitesse moteur, suivi de ligne.
 * Gains identifiés/validés en simulation : voir matlab/voiture_pid_tuning.m
 */
#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <Arduino.h>

class PIDController {
public:
    PIDController(float kp, float ki, float kd, float sortieMin, float sortieMax);

    // Doit etre appele a intervalle regulier fixe (dt en secondes)
    float calculer(float consigne, float mesure, float dt);

    void reinitialiser();
    void reglerGains(float kp, float ki, float kd);

private:
    float _kp, _ki, _kd;
    float _sortieMin, _sortieMax;
    float _integrale;
    float _erreurPrecedente;
    bool _premierAppel;
};

#endif // PID_CONTROLLER_H
