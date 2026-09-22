# Schémas de câblage

## 1. Voiture autonome (Arduino Uno/Nano)

```
                         +----------------------+
                         |      Arduino Uno      |
                         |                        |
  HC-SR04 --- TRIG ----->| D12                    |
  (avant)  -- ECHO ----->| D13                    |
                         |                        |
  IR gauche  -- OUT ---->| A0                     |
  IR centre  -- OUT ---->| A1                     |
  IR droite  -- OUT ---->| A2                     |
                         |                        |
  Servo scan -- signal ->| D11 (PWM)              |
                         |                        |
  Encodeur G -- signal ->| D2  (INT0)             |
  Encodeur D -- signal ->| D3  (INT1)             |
                         |                        |
                         |          L298N (pont H)|
  Moteur G ---- OUT1/OUT2 <--| IN1=D7 IN2=D8 ENA=D5(PWM)
  Moteur D ---- OUT3/OUT4 <--| IN3=D9 IN4=D10 ENB=D6(PWM)
                         |                        |
  Batterie 7.4V ---------|--- VIN (Arduino)       |
                     +---|--- +12V (L298N)        |
                     +---|--- GND commun           |
                         +------------------------+
```

**Points d'attention :**
- Masse commune obligatoire entre Arduino, L298N et batterie moteurs.
- Le HC-SR04 fonctionne en 5V ; alimenter depuis le Arduino (pas la
  batterie moteurs qui peut chuter sous charge).
- Les capteurs IR reflectifs doivent être placés à 1-2 cm du sol, espacés
  d'environ 1,5 cm entre eux (adapter selon la largeur de la ligne à suivre).

## 2. Drone autonome (quadricoptère, Arduino Uno/Nano)

```
                         +----------------------+
                         |      Arduino Uno      |
                         |                        |
  MPU6050 --- SDA ------>| A4                     |
           -- SCL ------>| A5                     |
           -- VCC ------>| 5V (via regulateur si besoin)
           -- GND ------>| GND                    |
                         |                        |
  Recepteur -- CH1 (roulis)  -> A0                |
  radio RC  -- CH2 (tangage) -> A1                |
              -- CH3 (lacet)   -> A2                |
              -- CH4 (gaz)     -> A3                |
                         |                        |
  ESC 1 (avant gauche) --| D3                     |
  ESC 2 (avant droit)  --| D5                     |
  ESC 3 (arriere gauche)| D6                     |
  ESC 4 (arriere droit)-| D9                     |
                         |                        |
  Diviseur tension LiPo -| A6 (mesure batterie)   |
                         +------------------------+

  Alimentation : LiPo 3S 11.1V --> ESC (BEC 5V) --> Arduino (via VIN ou 5V regule)
```

**Points d'attention :**
- **Hélices retirées** pendant tous les tests logiciels et le réglage des
  gains PID. Ne monter les hélices qu'après validation complète du
  comportement (moteurs bloqués, drone tenu à la main).
- Le sens de rotation des moteurs doit respecter le mixage `+`
  (avant-gauche et arrière-droit horaires ; avant-droit et arrière-gauche
  anti-horaires) — voir `drone/lib/Moteurs/Moteurs.cpp`.
- Le diviseur de tension sur A6 doit ramener 12,6V (LiPo 3S pleine
  charge) sous 5V : pont résistif 1:3 recommandé (ex. 20kΩ / 10kΩ).
- Toujours armer les ESC gaz à zéro (`moteurs.armer()`), sinon certains
  contrôleurs refusent de démarrer par sécurité.

## 3. Configuration en "+" du quadricoptère

```
                    Avant
                      |
        Moteur 1 (AVG)   Moteur 2 (AVD)
              \\             //
               \\           //
                \\         //
                 [  FC   ]
                //         \\
               //           \\
              //             \\
        Moteur 3 (ARG)   Moteur 4 (ARD)
                      |
                   Arriere
```

- Moteurs 1 et 4 : rotation horaire (CW)
- Moteurs 2 et 3 : rotation anti-horaire (CCW)
