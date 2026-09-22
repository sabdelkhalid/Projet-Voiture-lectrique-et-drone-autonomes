/**
 * config.h - Configuration materielle et parametres de reglage
 * Drone autonome (quadricoptere) - projet embarque Arduino
 *
 * Toutes les valeurs numeriques (PID, limites) sont justifiees dans
 * docs/01_rapport_dimensionnement.md et verifiees par simulation dans
 * matlab/drone_pid_stabilisation.m et matlab/drone_simulation_vol.m
 *
 * NB : sur ATmega328P (Uno/Nano), la boucle de stabilisation tourne a
 * environ 250 Hz (Ts ~ 4 ms). Un microcontroleur plus puissant (STM32,
 * Teensy) est recommande pour une frequence superieure et davantage de
 * capteurs (GPS, baro) simultanes, mais l'architecture logicielle reste
 * identique.
 */
#ifndef CONFIG_H
#define CONFIG_H

// ---------------------------------------------------------------------
// Affectation des broches - 4x ESC (signal PWM style servo, 1000-2000us)
// ---------------------------------------------------------------------
#define PIN_ESC_AVANT_G    3   // moteur 1 (avant gauche,  sens horaire)
#define PIN_ESC_AVANT_D    5   // moteur 2 (avant droit,   sens anti-horaire)
#define PIN_ESC_ARRIERE_G  6   // moteur 3 (arriere gauche, sens anti-horaire)
#define PIN_ESC_ARRIERE_D  9   // moteur 4 (arriere droit,  sens horaire)

// MPU6050 (IMU 6 axes) en I2C : SDA=A4, SCL=A5

// Recepteur radio (PWM, 4 voies : roulis, tangage, lacet, gaz)
#define PIN_RX_ROULIS      A0  // via pin-change interrupt ou pulseIn
#define PIN_RX_TANGAGE     A1
#define PIN_RX_LACET       A2
#define PIN_RX_GAZ         A3

// ---------------------------------------------------------------------
// Limites de commande ESC (microsecondes, format servo standard)
// ---------------------------------------------------------------------
#define ESC_MIN_US         1000
#define ESC_MAX_US         2000
#define ESC_ARMEMENT_US    1000   // valeur envoyee au demarrage (arme les ESC)

// ---------------------------------------------------------------------
// Frequence de boucle de stabilisation
// ---------------------------------------------------------------------
#define BOUCLE_PERIODE_MS  4      // 250 Hz (cf. rapport section 2.5)

// ---------------------------------------------------------------------
// Filtre complementaire (fusion accelerometre/gyroscope)
// ---------------------------------------------------------------------
#define FILTRE_ALPHA       0.98f

// ---------------------------------------------------------------------
// Gains PID - Boucle d'angle (lente, exterieure) - voir rapport 2.5
// ---------------------------------------------------------------------
#define PID_ANGLE_ROULIS_KP   4.5f
#define PID_ANGLE_ROULIS_KI   0.2f
#define PID_ANGLE_TANGAGE_KP  4.5f
#define PID_ANGLE_TANGAGE_KI  0.2f

// ---------------------------------------------------------------------
// Gains PID - Boucle de taux angulaire (rapide, interieure)
// ---------------------------------------------------------------------
#define PID_TAUX_ROULIS_KP    0.70f
#define PID_TAUX_ROULIS_KD    0.015f
#define PID_TAUX_TANGAGE_KP   0.70f
#define PID_TAUX_TANGAGE_KD   0.015f
#define PID_TAUX_LACET_KP     1.20f
#define PID_TAUX_LACET_KI     0.05f

// Limite de la consigne de taux issue de la boucle d'angle (anti-emballement)
#define TAUX_CONSIGNE_MAX_DEGS  300.0f

// ---------------------------------------------------------------------
// Securite
// ---------------------------------------------------------------------
#define ANGLE_LIMITE_SECURITE_DEG  45.0f  // au-dela : coupure moteurs
#define TIMEOUT_RADIO_MS           500    // perte de liaison -> failsafe
#define TENSION_BATTERIE_MIN_V     10.5f  // seuil alerte LiPo 3S (3.5V/cellule)
#define PIN_TENSION_BATTERIE       A6     // diviseur de tension (1/3)

#endif // CONFIG_H
