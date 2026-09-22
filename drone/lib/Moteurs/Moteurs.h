/**
 * Moteurs.h - Pilotage de 4 ESC brushless (signal PWM standard 1000-2000us)
 * Mixage quadricoptere en configuration "+ " (avant/arriere/gauche/droite)
 */
#ifndef MOTEURS_H
#define MOTEURS_H

#include <Arduino.h>
#include <Servo.h>

class MoteursQuadri {
public:
    void begin(uint8_t pinAvG, uint8_t pinAvD, uint8_t pinArG, uint8_t pinArD);

    // Arme les ESC (procedure requise par la plupart des controleurs) :
    // doit etre appele une fois au demarrage, manette des gaz au minimum
    void armer();

    // gaz : 0-100 (%) ; correctionRoulis/Tangage/Lacet : sorties PID en unites arbitraires
    // deja mises a l'echelle par l'appelant (cf. mixage dans le sketch principal)
    void appliquerMixage(float gaz, float corrRoulis, float corrTangage, float corrLacet);

    void arreterTout();

private:
    Servo _escAvG, _escAvD, _escArG, _escArD;
    void ecrireEsc(Servo& esc, float commandeUs);
};

#endif // MOTEURS_H
