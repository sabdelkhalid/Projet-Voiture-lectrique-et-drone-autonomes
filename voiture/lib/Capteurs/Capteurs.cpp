#include "Capteurs.h"

// =======================================================================
//  CapteurUltrason - HC-SR04
// =======================================================================
CapteurUltrason::CapteurUltrason(uint8_t pinTrig, uint8_t pinEcho)
    : _pinTrig(pinTrig), _pinEcho(pinEcho) {}

void CapteurUltrason::begin() {
    pinMode(_pinTrig, OUTPUT);
    pinMode(_pinEcho, INPUT);
    digitalWrite(_pinTrig, LOW);
}

float CapteurUltrason::mesurerDistanceCm() {
    // Impulsion de declenchement 10us
    digitalWrite(_pinTrig, LOW);
    delayMicroseconds(2);
    digitalWrite(_pinTrig, HIGH);
    delayMicroseconds(10);
    digitalWrite(_pinTrig, LOW);

    unsigned long duree = pulseIn(_pinEcho, HIGH, TIMEOUT_US);
    if (duree == 0) {
        return -1.0f; // timeout : pas d'obstacle detecte / erreur de mesure
    }
    // distance = (duree * vitesse_son) / 2 ; vitesse_son = 0.0343 cm/us
    float distance = duree * 0.0343f / 2.0f;
    return distance;
}

float CapteurUltrason::mesurerDistanceFiltreeCm(uint8_t nEchantillons) {
    // On collecte N mesures puis on prend la mediane : robuste aux
    // echos parasites / valeurs aberrantes (cf. analyse_capteurs.m)
    float echantillons[16];
    if (nEchantillons > 16) nEchantillons = 16;

    uint8_t nValides = 0;
    for (uint8_t i = 0; i < nEchantillons; i++) {
        float d = mesurerDistanceCm();
        if (d > 0) {
            echantillons[nValides++] = d;
        }
        delay(5); // laisser le capteur se stabiliser entre 2 mesures
    }
    if (nValides == 0) return -1.0f;

    // tri a bulles (nValides <= 16, cout negligeable)
    for (uint8_t i = 0; i < nValides - 1; i++) {
        for (uint8_t j = 0; j < nValides - i - 1; j++) {
            if (echantillons[j] > echantillons[j + 1]) {
                float tmp = echantillons[j];
                echantillons[j] = echantillons[j + 1];
                echantillons[j + 1] = tmp;
            }
        }
    }
    return echantillons[nValides / 2]; // mediane
}

// =======================================================================
//  CapteursLigne - 3x capteurs IR reflectifs
// =======================================================================
CapteursLigne::CapteursLigne(uint8_t pinGauche, uint8_t pinCentre, uint8_t pinDroite)
    : _pinG(pinGauche), _pinC(pinCentre), _pinD(pinDroite) {}

void CapteursLigne::begin() {
    pinMode(_pinG, INPUT);
    pinMode(_pinC, INPUT);
    pinMode(_pinD, INPUT);
}

void CapteursLigne::calibrer() {
    // Moyenne des lectures sur fond clair (hors ligne) pendant 1s,
    // le seuil est fixe a 1.5x cette moyenne (methode simple et robuste)
    long somme = 0;
    const int N = 50;
    for (int i = 0; i < N; i++) {
        somme += analogRead(_pinG) + analogRead(_pinC) + analogRead(_pinD);
        delay(10);
    }
    int moyenneFond = somme / (N * 3);
    _seuilNoir = moyenneFond * 3 / 2;
}

bool CapteursLigne::ligneDetectee() {
    int g = analogRead(_pinG);
    int c = analogRead(_pinC);
    int d = analogRead(_pinD);
    return (g > _seuilNoir) || (c > _seuilNoir) || (d > _seuilNoir);
}

float CapteursLigne::lireErreurPosition() {
    int g = analogRead(_pinG);
    int c = analogRead(_pinC);
    int d = analogRead(_pinD);

    bool bG = g > _seuilNoir;
    bool bC = c > _seuilNoir;
    bool bD = d > _seuilNoir;

    if (!bG && !bC && !bD) {
        return NAN; // ligne perdue -> l'appelant doit gerer (derniere erreur connue)
    }

    // Barycentre pondere des capteurs actifs : -1 (gauche) .. 0 (centre) .. +1 (droite)
    float poidsTotal = 0, sommePonderee = 0;
    if (bG) { sommePonderee += -1.0f * g; poidsTotal += g; }
    if (bC) { sommePonderee +=  0.0f * c; poidsTotal += c; }
    if (bD) { sommePonderee += +1.0f * d; poidsTotal += d; }

    return sommePonderee / poidsTotal;
}
