#include "Moteurs.h"
#include "../../src/config.h"

void MoteursQuadri::begin(uint8_t pinAvG, uint8_t pinAvD, uint8_t pinArG, uint8_t pinArD) {
    _escAvG.attach(pinAvG, ESC_MIN_US, ESC_MAX_US);
    _escAvD.attach(pinAvD, ESC_MIN_US, ESC_MAX_US);
    _escArG.attach(pinArG, ESC_MIN_US, ESC_MAX_US);
    _escArD.attach(pinArD, ESC_MIN_US, ESC_MAX_US);
}

void MoteursQuadri::armer() {
    _escAvG.writeMicroseconds(ESC_ARMEMENT_US);
    _escAvD.writeMicroseconds(ESC_ARMEMENT_US);
    _escArG.writeMicroseconds(ESC_ARMEMENT_US);
    _escArD.writeMicroseconds(ESC_ARMEMENT_US);
    delay(3000); // laisser le temps aux ESC de detecter le signal minimum et de "biper"
}

void MoteursQuadri::ecrireEsc(Servo& esc, float commandeUs) {
    commandeUs = constrain(commandeUs, ESC_MIN_US, ESC_MAX_US);
    esc.writeMicroseconds((int)commandeUs);
}

void MoteursQuadri::appliquerMixage(float gaz, float corrRoulis, float corrTangage, float corrLacet) {
    // Conversion du gaz (0-100%) en largeur d'impulsion de base
    float baseUs = ESC_MIN_US + (gaz / 100.0f) * (ESC_MAX_US - ESC_MIN_US);

    // Mixage standard quadri "+ " :
    //   Avant Gauche  (horaire)      : + tangage - roulis + lacet
    //   Avant Droit   (anti-horaire) : + tangage + roulis - lacet
    //   Arriere Gauche(anti-horaire) : - tangage - roulis - lacet
    //   Arriere Droit (horaire)      : - tangage + roulis + lacet
    float avG = baseUs + corrTangage - corrRoulis + corrLacet;
    float avD = baseUs + corrTangage + corrRoulis - corrLacet;
    float arG = baseUs - corrTangage - corrRoulis - corrLacet;
    float arD = baseUs - corrTangage + corrRoulis + corrLacet;

    ecrireEsc(_escAvG, avG);
    ecrireEsc(_escAvD, avD);
    ecrireEsc(_escArG, arG);
    ecrireEsc(_escArD, arD);
}

void MoteursQuadri::arreterTout() {
    ecrireEsc(_escAvG, ESC_MIN_US);
    ecrireEsc(_escAvD, ESC_MIN_US);
    ecrireEsc(_escArG, ESC_MIN_US);
    ecrireEsc(_escArD, ESC_MIN_US);
}
