/**
 * config.h - Configuration materielle et parametres de reglage
 * Voiture autonome - projet embarque Arduino
 *
 * Toutes les valeurs numeriques (PID, seuils) sont justifiees dans
 * docs/01_rapport_dimensionnement.md et verifiees par simulation dans
 * matlab/voiture_pid_tuning.m
 */
#ifndef CONFIG_H
#define CONFIG_H

// ---------------------------------------------------------------------
// Affectation des broches - Pont en H L298N (actionneurs)
// ---------------------------------------------------------------------
#define PIN_MOTEUR_G_IN1   7
#define PIN_MOTEUR_G_IN2   8
#define PIN_MOTEUR_G_PWM   5     // ENA (PWM Timer0)
#define PIN_MOTEUR_D_IN1   9
#define PIN_MOTEUR_D_IN2   10
#define PIN_MOTEUR_D_PWM   6     // ENB (PWM Timer0)

// ---------------------------------------------------------------------
// Capteurs
// ---------------------------------------------------------------------
#define PIN_TRIG_AVANT     12
#define PIN_ECHO_AVANT     13
#define PIN_IR_GAUCHE      A0
#define PIN_IR_CENTRE      A1
#define PIN_IR_DROITE      A2
#define PIN_SERVO_SCAN     11    // servo qui oriente le capteur ultrason

// MPU6050 en I2C : SDA=A4, SCL=A5 (Arduino Uno/Nano) - pas de define necessaire

// Encodeurs a fourche optiques (roues codeuses) - broches a interruption obligatoires
#define PIN_ENCODEUR_G     2   // INT0
#define PIN_ENCODEUR_D     3   // INT1
#define IMPULSIONS_PAR_TOUR 20 // resolution du disque code

// ---------------------------------------------------------------------
// Parametres physiques (issus du dimensionnement)
// ---------------------------------------------------------------------
#define RAYON_ROUE_M       0.0325f
#define VITESSE_MAX_MS     0.50f
#define DISTANCE_SECURITE_CM 20.0f   // seuil de detection d'obstacle
#define DISTANCE_ARRET_CM    8.0f    // arret d'urgence

// ---------------------------------------------------------------------
// Gains PID - Suivi de ligne (voir matlab/voiture_pid_tuning.m)
// ---------------------------------------------------------------------
#define PID_LIGNE_KP       40.0f
#define PID_LIGNE_KI       0.5f
#define PID_LIGNE_KD       12.0f
#define PID_LIGNE_TS_MS    20

// ---------------------------------------------------------------------
// Gains PI - Regulation de vitesse moteur (voir matlab/voiture_pid_tuning.m)
// ---------------------------------------------------------------------
#define PID_VITESSE_KP     0.45f
#define PID_VITESSE_KI     3.0f

// ---------------------------------------------------------------------
// Vitesse de base (PWM 0-255)
// ---------------------------------------------------------------------
#define PWM_BASE           150
#define PWM_MAX             255
#define PWM_MIN_MOUVEMENT    60   // en dessous, le moteur ne demarre pas (zone morte)

#endif // CONFIG_H
