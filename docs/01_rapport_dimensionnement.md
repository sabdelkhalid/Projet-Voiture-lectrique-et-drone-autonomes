# Rapport d'ingénierie — Dimensionnement

Ce document présente la démarche de dimensionnement (calculs justificatifs) utilisée
pour concevoir la voiture et le drone autonomes. Les valeurs numériques obtenues ici
alimentent directement les scripts MATLAB (`/matlab`) et les constantes du firmware
Arduino (`config.h` dans chaque sous-projet).

---

## 1. Voiture autonome

### 1.1 Hypothèses géométriques et massiques

| Paramètre | Symbole | Valeur |
|---|---|---|
| Masse totale (châssis + batterie + carte) | m | 1,20 kg |
| Diamètre des roues | d | 65 mm |
| Rayon des roues | r | 32,5 mm |
| Empattement | L | 150 mm |
| Voie | E | 130 mm |
| Coefficient de résistance au roulement (bitume/carrelage) | Crr | 0,02 |
| Vitesse max visée | v_max | 0,50 m/s |
| Accélération visée | a | 0,30 m/s² |

### 1.2 Couple moteur nécessaire

Force de résistance au roulement :
F_roll = m · g · Crr = 1,20 × 9,81 × 0,02 = **0,235 N**

Force d'accélération :
F_acc = m · a = 1,20 × 0,30 = **0,360 N**

Force totale à fournir (répartie sur 2 roues motrices) :
F_tot = F_roll + F_acc = 0,595 N → F_par_roue ≈ 0,298 N

Couple utile par roue :
T_roue = F_par_roue × r = 0,298 × 0,0325 = **9,68 mN·m**

Vitesse angulaire requise :
ω = v_max / r = 0,50 / 0,0325 = 15,38 rad/s → **146,9 tr/min**

**Conclusion :** un motoréducteur type TT (6 V, ~200 tr/min à vide, couple de
calage ≈ 0,80 N·cm = 8,0 mN·m à pleine charge... en pratique les moteurs TT du
commerce donnent un couple de calage bien supérieur, de l'ordre de 0,3–0,8 kg·cm
soit 30–80 mN·m) offre une marge de sécurité > 3, ce qui est cohérent avec les
pertes par friction interne et les à-coups de démarrage. Voir `matlab/calculs_dimensionnement.m`.

### 1.3 Autonomie énergétique

| Consommateur | Courant estimé |
|---|---|
| 4 × moteur TT en charge | 4 × 180 mA = 720 mA |
| Arduino Uno/Nano | 50 mA |
| Capteurs (2×HC-SR04, MPU6050, 3×IR) | 80 mA |
| Marge (driver moteur, LEDs) | 150 mA |
| **Total** | **≈ 1000 mA** |

Batterie : 2 × Li-ion 18650 en série, 7,4 V, 2000 mAh.

Autonomie théorique : t = C / I = 2000 / 1000 = 2,0 h
Autonomie réelle estimée (rendement électrique/mécanique ≈ 70 %) :
**t_réel ≈ 1,4 h (≈ 84 min)**

### 1.4 Régulation de vitesse — fonction de transfert moteur CC

Modèle électromécanique simplifié du moteur CC (inductance négligée) :

  J·dω/dt = Kt·i − Fv·ω − Tcharge
  L·di/dt ≈ 0  ⇒  i ≈ (V − Ke·ω) / R

Fonction de transfert vitesse/tension (1ᵉʳ ordre) :

  Ω(s) / V(s) = Kt / [(R·J)s + (R·Fv + Kt·Ke)]

Avec les paramètres identifiés expérimentalement (constante de temps mesurée
τ ≈ 0,15 s, gain statique K ≈ 25 (tr/min)/V), le correcteur PI est calculé par
placement de pôle (voir `matlab/voiture_pid_tuning.m`) :

  **Kp = 0,45  Ki = 3,0  Kd = 0 (régulation PI suffisante pour la vitesse)**

### 1.5 Boucle de suivi de ligne (asservissement de cap)

Erreur mesurée : e = position pondérée des 3 capteurs IR (−1, 0, +1)
Sortie : différentiel de PWM appliqué aux roues gauche/droite

Correcteur PID discret (Ts = 20 ms) réglé par essais successifs (méthode
Ziegler–Nichols en boucle fermée) puis affiné en simulation :

  **Kp = 40   Ki = 0,5   Kd = 12**

---

## 2. Drone autonome (quadricoptère)

### 2.1 Hypothèses

| Paramètre | Symbole | Valeur |
|---|---|---|
| Masse totale au décollage (MTOW) | m | 0,90 kg |
| Diamètre du cadre (diagonale) | — | 250 mm |
| Nombre de moteurs | — | 4 |
| Rapport poussée/poids visé | T/W | ≥ 2,0 |

### 2.2 Poussée nécessaire

Poids total : P = m·g = 0,90 × 9,81 = **8,83 N**

Poussée totale requise (T/W = 2) : T_tot = 2 × 8,83 = **17,66 N** ≈ 1800 g-force

Poussée par moteur : T_moteur = 17,66 / 4 = **4,41 N** ≈ 450 g

→ Choix : moteurs brushless 2204 2300 KV + hélices 5040, capables de développer
≈ 480–500 g de poussée chacun à pleine puissance sous 3S (mesure constructeur),
ce qui couvre la marge demandée.

### 2.3 Poussée au vol stationnaire (hover)

Poussée nécessaire au hover = poids = 8,83 N → 2,21 N/moteur (≈ 225 g/moteur),
soit un régime moteur estimé à **≈ 45–55 % du régime max** (zone de fonctionnement
la plus efficace, cohérent avec un bon choix de dimensionnement).

### 2.4 Autonomie de vol

Batterie : LiPo 3S, 11,1 V, 1500 mAh, 25C

Courant estimé au hover : I_hover ≈ 4 × 4,0 A = **16 A**

Autonomie de vol stationnaire :
t_hover = (C × 0,8) / I_hover = (1,5 Ah × 0,8) / 16 A × 60 = **4,5 min**

(le facteur 0,8 limite la décharge à 80 % de la capacité pour préserver la
batterie ; en vol de translation la consommation moyenne baisse légèrement,
autonomie réelle estimée **5 à 6 min**, cohérente avec les drones de cette
catégorie).

### 2.5 Boucle de stabilisation — architecture cascade

Deux boucles imbriquées, exécutées à 250 Hz (période 4 ms) :

1. **Boucle interne — vitesse angulaire** (gyroscope, rad/s) → correcteur PD rapide
2. **Boucle externe — angle** (fusion complémentaire accéléro + gyro, °) → correcteur PI lent

| Axe | Boucle angle (Kp, Ki) | Boucle taux (Kp, Kd) |
|---|---|---|
| Roulis (Roll) | 4,5 ; 0,2 | 0,70 ; 0,015 |
| Tangage (Pitch) | 4,5 ; 0,2 | 0,70 ; 0,015 |
| Lacet (Yaw) | 3,0 ; 0,1 | 1,20 ; 0,000 |

Valeurs obtenues par simulation dans `matlab/drone_pid_stabilisation.m`
(réponse indicielle : dépassement < 10 %, temps de stabilisation < 0,4 s).

### 2.6 Filtre complémentaire (fusion IMU)

  angle_k = α·(angle_(k-1) + gyro·Ts) + (1 − α)·angle_accel,  avec α = 0,98

Ce filtre simple (moins coûteux qu'un Kalman) est suffisant pour un
stabilisateur embarqué sur ATmega328P (voir `drone/lib/IMU`).

---

## 3. Synthèse des marges de sécurité

| Sous-système | Grandeur | Besoin | Capacité | Marge |
|---|---|---|---|---|
| Voiture — moteur | Couple | 9,7 mN·m | ≈ 30–80 mN·m | ×3 à ×8 |
| Voiture — batterie | Autonomie | — | ≈ 84 min | — |
| Drone — poussée | T/W | ≥ 2,0 | ≈ 2,0–2,2 | conforme |
| Drone — batterie | Autonomie hover | — | ≈ 4,5–6 min | — |

Tous les calculs sont reproductibles dans `matlab/calculs_dimensionnement.m`.
