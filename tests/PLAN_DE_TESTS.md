# Plan de tests et procédure de validation

Démarche de validation en 4 niveaux, de la simulation au produit fini,
conforme à une approche d'ingénierie classique (V-model simplifié).

## Niveau 1 — Simulation (MATLAB)

| Test | Script | Critère de succès | Résultat |
|---|---|---|---|
| Stabilité boucle vitesse moteur | `voiture_pid_tuning.m` | Dépassement < 5 % | ✅ 0,5 % |
| Suivi de ligne bruité | `voiture_pid_tuning.m` | RMSE < 1,0 (u.a.) | ✅ 0,41 |
| Évitement d'obstacle | `voiture_simulation_trajectoire.m` | Distance min > distance d'arrêt urgence | ✅ 0,32 m > 0,08 m |
| Rejet de perturbation drone | `drone_pid_stabilisation.m` | Retour à consigne < 1 s | ✅ < 0,5 s |
| Maintien altitude | `drone_simulation_vol.m` | Dépassement < 20 % | ✅ ≈ 12 % |

Voir `docs/03_resultats_simulation.md` pour le détail.

## Niveau 2 — Tests unitaires du firmware (banc de test, sans mécanique)

À réaliser moteurs/hélices débranchés :

1. **CapteurUltrason** : vérifier la cohérence de `mesurerDistanceCm()`
   avec un mètre ruban (tolérance ± 1 cm entre 5 et 200 cm).
2. **CapteursLigne** : vérifier `lireErreurPosition()` sur une ligne noire
   test (bande adhésive 2 cm) — valeur attendue proche de 0 quand centré.
3. **MoteurCC::commander()** : vérifier le sens de rotation pour des
   valeurs positives/négatives (multimètre ou oscilloscope sur ENA/ENB).
4. **IMU** : après `calibrer()`, les taux angulaires au repos doivent être
   proches de 0 °/s (± 1 °/s) ; les angles proches de 0° si le drone est
   posé à plat.
5. **MoteursQuadri::armer() / appliquerMixage()** : vérifier au servo-testeur
   ou oscilloscope que chaque sortie ESC varie bien entre 1000 et 2000 µs.

## Niveau 3 — Tests intégrés (banc fixe)

- Voiture posée sur cales (roues dans le vide) : vérifier que la boucle
  de suivi de ligne fait tourner les roues dans le bon sens pour une
  ligne déplacée manuellement sous les capteurs.
- Drone fixé sur un support à cardan (test bench) ou tenu à la main,
  **hélices retirées** : vérifier que les moteurs "avant" accélèrent
  quand on incline le drone vers l'arrière (correction de tangage), etc.

## Niveau 4 — Essais réels

- Voiture : parcours avec ligne et un obstacle unique, vitesse réduite
  (`PWM_BASE` abaissé) pour le premier essai.
- Drone : **essai en extérieur dégagé, hélices montées, avec un pilote
  aux commandes radio prêt à reprendre la main à tout instant.** Ne
  jamais lancer un essai en pilotage 100 % autonome sans avoir validé
  la stabilisation manuelle au préalable.

## Fichier de log d'exemple

`log_capteurs_exemple.csv` contient un enregistrement simulé (bruit +
valeurs aberrantes) au format attendu par `matlab/analyse_capteurs.m` :

```
temps_s,distance_cm
0.000,61.32
0.060,59.87
...
```

Pour enregistrer un vrai log depuis l'Arduino, ajouter dans la boucle
`SUIVI_LIGNE` un `Serial.print()` formaté à l'identique, puis rediriger
la sortie du moniteur série vers un fichier (ou utiliser un script
Python `pyserial` dédié — non fourni ici pour rester centré sur la
chaîne Arduino + MATLAB demandée).
