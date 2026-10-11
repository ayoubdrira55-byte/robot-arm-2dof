# Résultats

Validation du modèle : dérive d'énergie max = 4.0e-07 %

## Mouvement point à point (avec perturbation)

| Correcteur | Erreur RMS q1 [°] | Erreur RMS q2 [°] | Erreur max q2 [°] | Énergie [J] |
|---|---|---|---|---|
| PID | 3.512 | 3.007 | 10.574 | 63.3 |
| PD + compensation gravité | 2.236 | 2.139 | 9.347 | 63.5 |
| Couple calculé | 0.785 | 4.488 | 22.019 | 65.9 |

## Suivi de cercle (régime établi, t > 4 s)

| Correcteur | Erreur moyenne [mm] | Erreur max [mm] |
|---|---|---|
| PID | 39.45 | 101.12 |
| PD + compensation gravité | 8.59 | 15.10 |
| Couple calculé | 0.02 | 0.05 |
