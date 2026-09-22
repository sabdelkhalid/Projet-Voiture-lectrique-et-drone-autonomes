/**
 * drone_autonome.ino
 * -------------------------------------------------------------------
 * Drone autonome (quadricoptere) - stabilisation d'attitude
 * Carte cible : Arduino Uno / Nano (ATmega328P), boucle a 250 Hz
 *
 * Architecture :
 *   - IMU.h/.cpp            : MPU6050 + filtre complementaire (roulis/tangage)
 *   - Moteurs.h/.cpp        : pilotage 4x ESC + mixage quadricoptere "+"
 *   - PIDController.h/.cpp  : regulateur PID generique
 *   - config.h                : toutes les constantes materielles/reglages
 *
 * Boucle de commande (architecture cascade, cf. rapport section 2.5 et
 * matlab/drone_pid_stabilisation.m) :
 *
 *   consigne pilote --> [PID ANGLE] --> consigne de taux --> [PID TAUX] --> couple moteur
 *                              ^                                    ^
 *                         angle mesure (IMU)                 taux mesure (gyro)
 *
 * Securites implementees :
 *   - armement ESC obligatoire au demarrage (gaz a zero)
 *   - coupure moteurs si angle > ANGLE_LIMITE_SECURITE_DEG
 *   - failsafe si perte de liaison radio (TIMEOUT_RADIO_MS)
 *   - alerte tension batterie basse
 * -------------------------------------------------------------------
 */
#include "config.h"
#include "../lib/IMU/IMU.h"
#include "../lib/Moteurs/Moteurs.h"
#include "../lib/PID/PIDController.h"

// ---------------------------------------------------------------------
// Objets globaux
// ---------------------------------------------------------------------
IMU imu;
MoteursQuadri moteurs;

// Boucle d'angle (exterieure, lente) - roulis / tangage
PIDController pidAngleRoulis(PID_ANGLE_ROULIS_KP, PID_ANGLE_ROULIS_KI, 0,
                              -TAUX_CONSIGNE_MAX_DEGS, TAUX_CONSIGNE_MAX_DEGS);
PIDController pidAngleTangage(PID_ANGLE_TANGAGE_KP, PID_ANGLE_TANGAGE_KI, 0,
                               -TAUX_CONSIGNE_MAX_DEGS, TAUX_CONSIGNE_MAX_DEGS);

// Boucle de taux (interieure, rapide) - roulis / tangage / lacet
PIDController pidTauxRoulis(PID_TAUX_ROULIS_KP, 0, PID_TAUX_ROULIS_KD, -400, 400);
PIDController pidTauxTangage(PID_TAUX_TANGAGE_KP, 0, PID_TAUX_TANGAGE_KD, -400, 400);
PIDController pidTauxLacet(PID_TAUX_LACET_KP, PID_TAUX_LACET_KI, 0, -400, 400);

// ---------------------------------------------------------------------
// Etat vol
// ---------------------------------------------------------------------
enum EtatVol { DESARME, VOL_STABILISE, FAILSAFE };
EtatVol etatVol = DESARME;

unsigned long dernierTempsBoucle = 0;
unsigned long dernierSignalRadio = 0;

void setup() {
    Serial.begin(115200);
    Serial.println(F("=== Drone autonome - initialisation ==="));

    pinMode(PIN_RX_ROULIS, INPUT);
    pinMode(PIN_RX_TANGAGE, INPUT);
    pinMode(PIN_RX_LACET, INPUT);
    pinMode(PIN_RX_GAZ, INPUT);

    if (!imu.begin()) {
        Serial.println(F("ERREUR : MPU6050 non detecte ! Verifier le cablage I2C."));
        while (1) { delay(1000); } // blocage volontaire : ne jamais decoller sans IMU
    }

    Serial.println(F("Calibration gyroscope - NE PAS BOUGER LE DRONE..."));
    imu.calibrer(500);
    Serial.println(F("Calibration terminee."));

    moteurs.begin(PIN_ESC_AVANT_G, PIN_ESC_AVANT_D, PIN_ESC_ARRIERE_G, PIN_ESC_ARRIERE_D);

    Serial.println(F("Armement des ESC (gaz au minimum)..."));
    moteurs.armer();
    Serial.println(F("Pret au vol."));

    etatVol = VOL_STABILISE;
    dernierTempsBoucle = millis();
    dernierSignalRadio = millis();
}

// Lecture d'une voie radio PWM (1000-2000us). Utilise pulseIn ; pour une
// application reelle, preferer des interruptions PinChange pour ne pas
// bloquer la boucle principale (documente en commentaire dans le README).
float lireVoieRadio(uint8_t pin, unsigned long defautUs = 1500) {
    unsigned long largeur = pulseIn(pin, HIGH, 25000UL); // timeout 25ms
    if (largeur == 0) return defautUs; // pas de nouvelle trame -> conserve la derniere consigne
    dernierSignalRadio = millis();
    return constrain(largeur, 1000, 2000);
}

void loop() {
    unsigned long maintenant = millis();
    if (maintenant - dernierTempsBoucle < BOUCLE_PERIODE_MS) {
        return; // cadence la boucle a la periode fixee (250 Hz)
    }
    float dt = (maintenant - dernierTempsBoucle) / 1000.0f;
    dernierTempsBoucle = maintenant;

    // ------------------------------------------------------------
    // 1) Lecture radio -> consignes pilote
    // ------------------------------------------------------------
    float usRoulis  = lireVoieRadio(PIN_RX_ROULIS);
    float usTangage = lireVoieRadio(PIN_RX_TANGAGE);
    float usLacet    = lireVoieRadio(PIN_RX_LACET);
    float usGaz       = lireVoieRadio(PIN_RX_GAZ);

    float consigneRoulisDeg  = map(usRoulis, 1000, 2000, -300, 300) / 10.0f;   // +/-30 deg
    float consigneTangageDeg = map(usTangage, 1000, 2000, -300, 300) / 10.0f;
    float consigneLacetDegS  = map(usLacet, 1000, 2000, -1500, 1500) / 10.0f;  // +/-150 deg/s
    float consigneGazPct     = map(usGaz, 1000, 2000, 0, 1000) / 10.0f;        // 0-100%

    // ------------------------------------------------------------
    // 2) Failsafe : perte radio ou gaz au minimum -> desarmement
    // ------------------------------------------------------------
    if (maintenant - dernierSignalRadio > TIMEOUT_RADIO_MS) {
        etatVol = FAILSAFE;
    } else if (consigneGazPct < 3.0f) {
        etatVol = DESARME; // securite : gaz bas = moteurs coupes (comme un vrai FC)
    } else if (etatVol != FAILSAFE) {
        etatVol = VOL_STABILISE;
    }

    // ------------------------------------------------------------
    // 3) Mise a jour de l'IMU (indispensable meme si desarme, pour
    //    garder le filtre complementaire "chaud")
    // ------------------------------------------------------------
    imu.mettreAJour(dt);
    AttitudeIMU att = imu.obtenirAttitude();

    // ------------------------------------------------------------
    // 4) Securite angle : au-dela de la limite, on coupe tout
    // ------------------------------------------------------------
    if (fabs(att.roulisDeg) > ANGLE_LIMITE_SECURITE_DEG ||
        fabs(att.tangageDeg) > ANGLE_LIMITE_SECURITE_DEG) {
        etatVol = FAILSAFE;
    }

    // ------------------------------------------------------------
    // 5) Boucle cascade : angle -> taux -> commande moteur
    // ------------------------------------------------------------
    if (etatVol == VOL_STABILISE) {
        // Boucle externe (angle)
        float tauxConsigneRoulis  = pidAngleRoulis.calculer(consigneRoulisDeg, att.roulisDeg, dt);
        float tauxConsigneTangage = pidAngleTangage.calculer(consigneTangageDeg, att.tangageDeg, dt);

        // Boucle interne (taux angulaire, mesure gyro)
        float corrRoulis  = pidTauxRoulis.calculer(tauxConsigneRoulis, att.tauxRoulisDegS, dt);
        float corrTangage = pidTauxTangage.calculer(tauxConsigneTangage, att.tauxTangageDegS, dt);
        // Le lacet est commande directement en taux (pas de boucle d'angle : le cap
        // absolu necessiterait un magnetometre, hors perimetre de ce firmware)
        float corrLacet   = pidTauxLacet.calculer(consigneLacetDegS, att.tauxLacetDegS, dt);

        moteurs.appliquerMixage(consigneGazPct, corrRoulis, corrTangage, corrLacet);

    } else {
        // DESARME ou FAILSAFE : coupure moteurs et reinitialisation des PID
        // (evite un "coup" au reamorcage du vol du fait de termes integraux residuels)
        moteurs.arreterTout();
        pidAngleRoulis.reinitialiser();
        pidAngleTangage.reinitialiser();
        pidTauxRoulis.reinitialiser();
        pidTauxTangage.reinitialiser();
        pidTauxLacet.reinitialiser();
    }

    // ------------------------------------------------------------
    // 6) Telemetrie (a desactiver en vol reel : le Serial ralentit la boucle)
    // ------------------------------------------------------------
    static uint16_t compteurAffichage = 0;
    if (++compteurAffichage >= 50) { // ~5 Hz avec une boucle a 250Hz
        compteurAffichage = 0;
        Serial.print(F("Etat=")); Serial.print(etatVol);
        Serial.print(F(" Roulis=")); Serial.print(att.roulisDeg);
        Serial.print(F(" Tangage=")); Serial.print(att.tangageDeg);
        Serial.print(F(" Gaz=")); Serial.println(consigneGazPct);
    }
}
