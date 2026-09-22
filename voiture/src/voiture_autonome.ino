/**
 * voiture_autonome.ino
 * -------------------------------------------------------------------
 * Voiture autonome - suivi de ligne avec evitement d'obstacle
 * Carte cible : Arduino Uno / Nano (ATmega328P)
 *
 * Architecture :
 *   - Capteurs.h/.cpp      : acquisition HC-SR04 + 3x IR reflectifs
 *   - Actionneurs.h/.cpp   : pilotage moteurs CC via pont en H L298N
 *   - PIDController.h/.cpp : regulateur PID generique (ligne + vitesse)
 *   - config.h              : toutes les constantes materielles/reglages
 *
 * Boucles de commande :
 *   1) Suivi de ligne (PID, Ts=20ms) -> differentiel de PWM gauche/droite
 *   2) Regulation de vitesse (PI, encodeurs a fourche) -> compense la
 *      chute de tension batterie pour garder une vitesse constante
 *   3) Evitement d'obstacle (HC-SR04 + servo de scan) -> prioritaire,
 *      interrompt le suivi de ligne le temps de contourner l'obstacle
 *
 * Les gains PID sont ceux valides par simulation dans
 * matlab/voiture_pid_tuning.m (voir docs/01_rapport_dimensionnement.md).
 * -------------------------------------------------------------------
 */
#include <Servo.h>
#include "config.h"
#include "../lib/Capteurs/Capteurs.h"
#include "../lib/Actionneurs/Actionneurs.h"
#include "../lib/PID/PIDController.h"

// ---------------------------------------------------------------------
// Objets globaux
// ---------------------------------------------------------------------
CapteurUltrason  capteurUS(PIN_TRIG_AVANT, PIN_ECHO_AVANT);
CapteursLigne    capteursLigne(PIN_IR_GAUCHE, PIN_IR_CENTRE, PIN_IR_DROITE);

MoteurCC moteurGauche(PIN_MOTEUR_G_IN1, PIN_MOTEUR_G_IN2, PIN_MOTEUR_G_PWM);
MoteurCC moteurDroit(PIN_MOTEUR_D_IN1, PIN_MOTEUR_D_IN2, PIN_MOTEUR_D_PWM);
Chassis  chassis(moteurGauche, moteurDroit);

Servo servoScan;

PIDController pidLigne(PID_LIGNE_KP, PID_LIGNE_KI, PID_LIGNE_KD, -150, 150);
PIDController pidVitesse(PID_VITESSE_KP, PID_VITESSE_KI, 0, -80, 80);

// ---------------------------------------------------------------------
// Etat machine a etats
// ---------------------------------------------------------------------
enum EtatRobot { SUIVI_LIGNE, OBSTACLE_DETECTE, EVITEMENT, ARRET_URGENCE };
EtatRobot etat = SUIVI_LIGNE;

// ---------------------------------------------------------------------
// Odometrie (encodeurs a fourche, interruptions)
// ---------------------------------------------------------------------
volatile uint32_t compteurImpulsionsG = 0;
volatile uint32_t compteurImpulsionsD = 0;

void isrEncodeurG() { compteurImpulsionsG++; }
void isrEncodeurD() { compteurImpulsionsD++; }

// ---------------------------------------------------------------------
// Variables de boucle
// ---------------------------------------------------------------------
unsigned long dernierTempsLigne = 0;
unsigned long dernierTempsVitesse = 0;
float derniereErreurLigneConnue = 0;

const unsigned long PERIODE_VITESSE_MS = 100;

void setup() {
    Serial.begin(9600);
    Serial.println(F("=== Voiture autonome - initialisation ==="));

    capteurUS.begin();
    capteursLigne.begin();
    chassis.begin();
    servoScan.attach(PIN_SERVO_SCAN);
    servoScan.write(90); // capteur oriente vers l'avant

    pinMode(PIN_ENCODEUR_G, INPUT_PULLUP);
    pinMode(PIN_ENCODEUR_D, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_ENCODEUR_G), isrEncodeurG, RISING);
    attachInterrupt(digitalPinToInterrupt(PIN_ENCODEUR_D), isrEncodeurD, RISING);

    Serial.println(F("Calibration des capteurs de ligne (poser sur fond clair)..."));
    delay(1000);
    capteursLigne.calibrer();
    Serial.println(F("Calibration terminee. Demarrage dans 2s..."));
    delay(2000);
}

void loop() {
    unsigned long maintenant = millis();

    // ------------------------------------------------------------
    // 1) Lecture capteur de distance (chaque cycle, non filtree pour
    //    la reactivite ; une mesure filtree est utilisee pour confirmer
    //    un arret d'urgence, cf. plus bas)
    // ------------------------------------------------------------
    float distance = capteurUS.mesurerDistanceCm();

    if (distance > 0 && distance < DISTANCE_ARRET_CM) {
        etat = ARRET_URGENCE;
    } else if (distance > 0 && distance < DISTANCE_SECURITE_CM) {
        if (etat == SUIVI_LIGNE) etat = OBSTACLE_DETECTE;
    } else if (etat == OBSTACLE_DETECTE) {
        etat = SUIVI_LIGNE; // obstacle disparu avant manoeuvre engagee
    }

    // ------------------------------------------------------------
    // 2) Machine a etats
    // ------------------------------------------------------------
    switch (etat) {

    case ARRET_URGENCE: {
        chassis.stop();
        // Confirmation par mesure filtree (rejet des faux positifs)
        float distConfirmee = capteurUS.mesurerDistanceFiltreeCm(5);
        Serial.print(F("ARRET URGENCE - distance="));
        Serial.println(distConfirmee);
        if (distConfirmee < 0 || distConfirmee > DISTANCE_ARRET_CM) {
            etat = SUIVI_LIGNE;
        }
        delay(50);
        break;
    }

    case OBSTACLE_DETECTE: {
        chassis.stop();
        delay(150);
        Serial.println(F("Obstacle detecte -> scan gauche/droite"));

        servoScan.write(150); delay(300);
        float distGauche = capteurUS.mesurerDistanceFiltreeCm(3);
        servoScan.write(30);  delay(300);
        float distDroite = capteurUS.mesurerDistanceFiltreeCm(3);
        servoScan.write(90);  delay(200);

        Serial.print(F("Distance gauche=")); Serial.print(distGauche);
        Serial.print(F("  Distance droite=")); Serial.println(distDroite);

        // On part du cote le plus degage
        if ((distGauche < 0 || distGauche > distDroite)) {
            chassis.pivoterSurPlace(-120); // pivote a gauche
        } else {
            chassis.pivoterSurPlace(120);  // pivote a droite
        }
        delay(400);
        chassis.stop();
        etat = EVITEMENT;
        break;
    }

    case EVITEMENT: {
        // Avance tout droit pendant un court instant pour depasser l'obstacle,
        // puis retourne en mode suivi de ligne (qui recapturera la ligne si
        // elle est de nouveau detectee).
        chassis.deplacer(PWM_BASE, 0);
        delay(700);
        chassis.stop();
        pidLigne.reinitialiser();
        etat = SUIVI_LIGNE;
        break;
    }

    case SUIVI_LIGNE:
    default: {
        if (maintenant - dernierTempsLigne >= PID_LIGNE_TS_MS) {
            float dt = (maintenant - dernierTempsLigne) / 1000.0f;
            dernierTempsLigne = maintenant;

            float erreur = capteursLigne.lireErreurPosition();
            if (isnan(erreur)) {
                // Ligne perdue : on continue brievement dans la derniere
                // direction connue plutot que de s'arreter brutalement
                erreur = derniereErreurLigneConnue;
            } else {
                derniereErreurLigneConnue = erreur;
            }

            float correction = pidLigne.calculer(0.0f, erreur, dt);
            chassis.deplacer(PWM_BASE, (int16_t)correction);
        }
        break;
    }
    }

    // ------------------------------------------------------------
    // 3) Regulation de vitesse (PI) - a titre indicatif / extension :
    //    ajuste PWM_BASE effectif pour compenser la chute de tension
    //    batterie. Affichage diagnostic uniquement ici (non applique
    //    au chassis pour garder l'exemple simple et lisible).
    // ------------------------------------------------------------
    if (maintenant - dernierTempsVitesse >= PERIODE_VITESSE_MS) {
        float dt = (maintenant - dernierTempsVitesse) / 1000.0f;
        dernierTempsVitesse = maintenant;

        noInterrupts();
        uint32_t impG = compteurImpulsionsG;
        uint32_t impD = compteurImpulsionsD;
        compteurImpulsionsG = 0;
        compteurImpulsionsD = 0;
        interrupts();

        float rpmG = (impG / (float)IMPULSIONS_PAR_TOUR) * (60.0f / dt);
        float rpmD = (impD / (float)IMPULSIONS_PAR_TOUR) * (60.0f / dt);

        // Exemple d'utilisation du PI vitesse (consigne 60 tr/min) :
        // float correctionVitesse = pidVitesse.calculer(60.0f, (rpmG+rpmD)/2.0f, dt);

        Serial.print(F("RPM G=")); Serial.print(rpmG);
        Serial.print(F("  RPM D=")); Serial.println(rpmD);
    }
}
