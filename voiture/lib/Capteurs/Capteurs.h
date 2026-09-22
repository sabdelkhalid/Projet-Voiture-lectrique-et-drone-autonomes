/**
 * Capteurs.h - Acquisition des capteurs de la voiture autonome
 *   - HC-SR04 (ultrason) : mesure de distance frontale
 *   - 3x capteurs IR reflectifs : suivi de ligne
 *
 * Filtrage : mediane glissante + moyenne glissante (cf. matlab/analyse_capteurs.m)
 */
#ifndef CAPTEURS_H
#define CAPTEURS_H

#include <Arduino.h>

class CapteurUltrason {
public:
    CapteurUltrason(uint8_t pinTrig, uint8_t pinEcho);
    void begin();
    // Mesure brute (une acquisition), retourne -1 si timeout (pas d'echo)
    float mesurerDistanceCm();
    // Mesure filtree (mediane sur N echantillons) - plus robuste aux aberrants
    float mesurerDistanceFiltreeCm(uint8_t nEchantillons = 5);

private:
    uint8_t _pinTrig, _pinEcho;
    static const uint32_t TIMEOUT_US = 30000UL; // ~5m max
};

class CapteursLigne {
public:
    CapteursLigne(uint8_t pinGauche, uint8_t pinCentre, uint8_t pinDroite);
    void begin();
    // Etalonnage : a appeler au demarrage, ligne noire sur fond clair
    void calibrer();
    // Retourne l'erreur de position normalisee entre -1 (tout a gauche)
    // et +1 (tout a droite), 0 = ligne centree. Retourne NAN si ligne perdue.
    float lireErreurPosition();
    bool ligneDetectee();

private:
    uint8_t _pinG, _pinC, _pinD;
    int _seuilNoir = 500; // valeur ADC (0-1023), ajustee par calibrer()
};

#endif // CAPTEURS_H
