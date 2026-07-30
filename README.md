# Commande FOC - Carte OwnVerter (moteur, ID 5)

Firmware de contrôle de position d'un moteur synchrone (PMSM) par commande vectorielle
(Field Oriented Control), sur carte **OwnTech OwnVerter**. La consigne de position arrive
par un bus **RS485** depuis une IHM (Raspberry Pi + joystick).

Deux sources de position sont désormais disponibles et **commutables** : capteurs **Hall + PLL**
(par défaut, fonctionnel) ou **encodeur incrémental** (câblé, mais à calibrer — voir § 9).

> Ce fichier documente le code tel qu'il est écrit. Les points marqués **/!\** signalent des
> comportements réels du code qui peuvent surprendre, ou du code présent mais inactif/à finaliser.

---

## 1. Vue d'ensemble

```
   IHM (Raspberry Pi)                      Carte OwnVerter
  ┌──────────────────┐                   ┌──────────────────────────┐
  │ joystick (ADC)   │   RS485           │ rs485_rx_callback        │
  │  → longueur fil  │──── 6 octets ────▶│  → theta_m_ref [rad]     │
  └──────────────────┘   115200 bauds    │         ↓                │
                                         │ control_torque()  10 kHz │
                                         │  → Idq_ref.q → FOC       │
                                         │  → Vabc → rapports cyc.  │
                                         └───────────┬──────────────┘
                                                     ▼
                              3 bras (LEG1/2/3) → moteur
                              position ← Hall+PLL  OU  encodeur (use_encoder)
```

Une carte = **un** moteur triphasé (3 bras de puissance). Pour deux moteurs, il faut deux
cartes sur le même bus RS485, différenciées par `MY_INVERTER_ID`.

---

## 2. Matériel

| Élément | Détail |
|---|---|
| Carte | OwnTech OwnVerter (mode Buck, 3 bras) |
| Moteur | PMSM 8 pôles → `pole_pairs = 4` |
| Capteurs position | 3 capteurs à effet Hall sur `PC6`, `PC7`, `PD2` |
| Encodeur | incrémental sur `TIMER3` — **lu et sélectionnable**, mais à calibrer |
| Communication | RS485, 115200 bauds, trames de 6 octets |
| Bus DC | seuil minimal d'armement `V_HIGH_MIN = 5.0 V` |

### Mécanique (constantes du code)

```c
capstan_radius = 1.5 cm     capstan_gear = 45     motor_gear = 15
line_cm_to_motor_rad = capstan_gear / (capstan_radius * motor_gear) = 2.0
```

→ **1 cm de fil = 2 rad à l'axe moteur.** Facteur unique de conversion entre la longueur de fil
envoyée par l'IHM et la consigne angulaire interne `theta_m_ref`.

---

## 3. Architecture logicielle - 3 tâches

| Tâche | Période | Rôle |
|---|---|---|
| `loop_critical_task()` | **100 µs (10 kHz)** | Mesures, position, observateur, FOC, rapports cycliques |
| `application_task()` | 250 ms | Affichage terminal, machine d'états, handshake RS485 |
| `loop_background_task()` | bloquante | Lecture des touches clavier (`console_getchar()`) |
| `rs485_rx_callback()` | sur réception | Décodage d'une trame RS485 → `theta_m_ref` |

Toutes créées dans `setup_routine()`, qui initialise aussi la puissance, les capteurs, le RS485,
les GPIO Hall, **l'encodeur** (`startLogIncrementalEncoder(TIMER3)`) et ScopeMimicry.

---

## 4. Protocole RS485

### Format de trame (6 octets, sans bourrage grâce à `#pragma pack(1)`)

**IHM → carte** (`InverterPayload_t`) :

| Offset | Champ | Type | Rôle |
|---|---|---|---|
| 0 | `target_id` | `uint8_t` | Adresse destinataire (4 ou 5) |
| 1–4 | `target_line_length` | `float` | Longueur de fil demandée, en **cm** |
| 5 | `checksum` | `uint8_t` | XOR des 5 octets précédents |

**Carte → IHM** (`InverterReply_t`) : `source_id` + `msg_code` (111 = démarrage) + `checksum`.

Un champ `mode` (interrupteur 3 positions) est **présent en commentaire** dans la structure et le
callback, non activé à ce jour.

### Adressage

```c
#define MY_INVERTER_ID 5   // LEFT_MOTOR = 5, RIGHT_MOTOR = 4
```

Sur un bus RS485, **toutes** les cartes reçoivent **toutes** les trames. C'est le test
`payload->target_id == MY_INVERTER_ID` qui fait qu'une carte n'applique que la trame qui lui est
destinée. Pour la deuxième carte : recompiler avec `MY_INVERTER_ID 4`.

### Checksum

```c
static uint8_t compute_checksum(const uint8_t* data, int len) {
    uint8_t cs = 0;
    for (int i = 0; i < len; i++) cs ^= data[i];
    return cs;
}
```

En réception, une trame dont le XOR ne correspond pas est **rejetée** (`return`), donc `theta_m_ref`
**garde sa valeur précédente**. Le checksum **détecte** la corruption, il ne la **corrige pas**.

> **/!\** Le checksum de la **réponse** est calculé avec `sizeof(InverterPayload_t) - 1`. C'est correct
> aujourd'hui car les deux structures font 6 octets ; à remplacer par `sizeof(InverterReply_t) - 1`
> pour rester juste si une structure change de taille (ex. ajout de l'octet `mode`).

### Handshake

Tant que `handshake_complete == false`, la carte réémet le code 111 toutes les 250 ms. Le drapeau
passe à `true` à la **première trame valide** reçue.

---

## 5. Mesure de position et de vitesse - `get_position_and_speed()`

Exécutée à 10 kHz. **Les deux sources sont calculées à chaque cycle**, puis une est sélectionnée.

### Chemin Hall + PLL (défaut)

1. Lecture des 3 Hall → `angle_index = HALL1 + 2*HALL2 + 4*HALL3` (1 à 6 valides).
2. Table de secteurs → angle électrique brut par pas de 60° :
   `hall_angle = ot_modulo_2pi(PI/3 * sector[angle_index] + k_angle_offset)`.
   `k_angle_offset = PI` est le **calage électrique** Hall ↔ bobinages.
3. **PLL** → `angle_filtered`, angle continu interpolé (les Hall seuls ne donnent qu'un escalier).
4. **Déroulement (unwrap)** → `angle_elec_unwrapped`, angle électrique cumulé sans saut à ±π.
5. `theta_m_hall = angle_elec_unwrapped / pole_pairs`.

### Chemin encodeur

```c
encoder_count   = (int32_t)spin.timer.getIncrementalEncoderValue(TIMER3);
theta_m_encoder = 2*PI * encoder_count / ENCODER_COUNTS_PER_REV;
theta_e_encoder = ot_modulo_2pi(pole_pairs * theta_m_encoder + encoder_offset);
```

### Sélection

```c
theta_m = use_encoder ? theta_m_encoder : theta_m_hall;
omega_m = w_mes_filter.calculateWithReturn(pllDatas.w) / pole_pairs;  // vitesse : PLL dans les 2 modes
```

> **/!\ La vitesse `omega_m` vient toujours de la PLL**, même en mode encodeur. Dériver l'encodeur
> donnerait une vitesse trop bruitée pour le terme `-K_posd*omega_m`. C'est un choix pragmatique
> pour un premier test : `omega_m` ne suit pas l'encodeur.

`theta_m` est un angle **mécanique cumulé** (multi-tours), indispensable pour asservir une longueur
de fil.

---

## 6. Boucle de commande - `control_torque()`

### Sélection de l'angle de commutation

```c
angle_4_control = use_encoder ? theta_e_encoder : angle_filtered;
```

C'est l'angle utilisé pour les transformées de Park (directe et inverse) - le coeur du FOC.

### Étage 1 : position → couple

```c
float32_t pos_error = theta_m - theta_m_ref;
int_pos += pos_error * Ts * anti_windup;                       // intégrateur
Idq_ref.q = -K_posi*int_pos - K_posp*theta_m - K_posd*omega_m;
```

Retour d'état avec action intégrale. Gains par défaut :

| Gain | Variable | Valeur | Agit sur |
|---|---|---|---|
| `K_posi` | intégral | 15.13 | erreur de position intégrée |
| `K_posp` | proportionnel | 3.72 | position `theta_m` |
| `K_posd` | dérivé | 0.208 | vitesse `omega_m` |

> **/!\ Seul le terme intégral voit `theta_m_ref`.** Les termes P et D agissent sur `theta_m` et
> `omega_m` **absolus**. Le suivi de consigne en régime établi repose donc entièrement sur
> l'intégrateur - d'où la forte dépendance à `K_posi`.

**Anti-windup :** si `Idq_ref.q` sature à ±`Iq_max` (5.0 A), `anti_windup = 0` gèle l'intégrateur.

### Étage 2 : FOC (courant)

```c
Idq  = Transform::to_dqo(Iabc, angle_4_control);      // Park : abc → dq
Vdq.d = pi_d.calculateWithReturn(Idq_ref.d, Idq.d);   // Idq_ref.d = 0 (pas d'affaiblissement)
Vdq.q = pi_q.calculateWithReturn(Idq_ref.q, Idq.q);
Vabc = Transform::to_threephase(Vdq, angle_4_control); // Park inverse : dq → abc
```

PID identiques d/q : `Kp = 1.4`, `Ti = 0.002029`, bornes ±12 V (= `MIN_DC_VOLTAGE × 0.4`).

### Étage 3 : rapports cycliques

```c
duty_abc.x = Vabc.x / V_high_filtered + 0.5;   // 0.5 = point milieu
```

Puis `apply_duties()` → `LEG1/2/3`.

---

## 7. Machine d'états

| État | Valeur | Comportement |
|---|---|---|
| `OFFSET_ST` | 0 | Calibration de l'offset des capteurs de courant (2000 cycles ≈ 0,2 s). PWM arrêté. |
| `IDLE_ST` | 1 | Au repos, PWM arrêté. Armement si `asked_mode == POWERMODE` **et** `V_high_filtered > 5 V`. |
| `POWER_ST` | 2 | Commande active, PWM en marche. |
| `ERROR_ST` | 3 | Surintensité détectée. PWM arrêté. Sortie uniquement par `'i'`. |

**/!\ Configuration actuelle (mode test) :** deux modifications court-circuitent l'armement manuel :

```c
asked_mode = POWERMODE;      // dans init_variables()          (au lieu de IDLEMODE)
control_state = POWER_ST;    // fin de OFFSET_ST               (au lieu de IDLE_ST)
```

→ **La carte s'arme automatiquement ~0,2 s après le flash.** La calibration d'offset est faite
avant. Mais ce chemin **ne vérifie pas `V_high_filtered`** (le test n'existe que dans
`IDLE_ST → POWER_ST`, contournée). À remettre en `IDLE_ST` pour un fonctionnement normal (§ 12).

**/!\ Au démarrage, `theta_m_ref` vaut 0** alors que `theta_m` vaut la position réelle : le moteur
part rejoindre la position zéro et peut bouger brusquement. Pour démarrer sans à-coup, décommenter
`theta_m_ref = theta_m;` dans `IDLE_ST`.

---

## 8. Sécurités

| Protection | Seuil | Mécanisme |
|---|---|---|
| Surintensité AC (I1, I2) | ±13 A | `error_counter++` |
| Surintensité DC (I_high) | 13 A | `error_counter++` |
| Filtrage faux positifs | - | `error_counter--` tous les 1000 cycles ; `ERROR_ST` au-delà de 3 |
| Saturation de couple | `Iq_max = 5.0 A` | Écrêtage de `Idq_ref.q` + anti-windup |
| Saturation de tension | ±12 V | Bornes des PID d/q |
| Trame corrompue | XOR | Trame rejetée, consigne figée |

> **Aucun arrêt d'urgence logiciel indépendant du terminal.** Le seul arrêt est la touche `'i'`.
> Pour une utilisation hors banc, prévoir un **arrêt d'urgence matériel** coupant la puissance sans
> dépendre du logiciel ni du bus RS485.

---

## 9. Mode encodeur — état et calibration /!\

Le chemin encodeur est maintenant **câblé** (lecture active, sélection par `use_encoder`, bascule
par la touche `'c'`), mais **pas encore utilisable en confiance** : trois paramètres ne sont pas
validés dans le code.

| Paramètre | État | Conséquence si faux |
|---|---|---|
| `encoder_offset = -0.1F` | **valeur d'essai** | Angle électrique décalé → en mode encodeur le FOC applique la tension au mauvais angle → le moteur cale, s'emballe, ou déclenche la surintensité. **Bloquant.** |
| `ENCODER_COUNTS_PER_REV` | `100*4/(45/15) = 133,33 → 133` | Mauvaise mise à l'échelle de `theta_m_encoder` : « un tour d'encodeur » ≠ longueur de fil attendue. Dépend de la résolution réelle et du point de montage (axe moteur ou capstan). **Bloquant pour l'échelle.** |
| Bascule à chaud | non protégée | Changer la source de l'angle pendant `POWER_ST` avec un offset imparfait → à-coup violent. |

### Procédure de calibration (à faire en mode Hall, qui fonctionne)

1. Démarrer en mode Hall (`use_encoder = false`, défaut).
2. Faire tourner lentement le moteur et lire dans la télémétrie `theta_e_encoder` **et**
   `angle_filtered` côte à côte.
3. Ajuster `encoder_offset` jusqu'à ce que les deux angles **se superposent**.
4. Si les deux angles **divergent progressivement** au lieu de rester superposés → `ENCODER_COUNTS_PER_REV`
   est faux (erreur d'échelle) : corriger la résolution.
5. Quand ils coïncident : passer en `IDLE_ST`, presser `'c'`, puis réarmer.

> **N'appuyer sur `'c'` qu'en IDLE, jamais en POWER.**

---

## 10. Code présent mais **inactif**

| Élément | État | Détail |
|---|---|---|
| **Observateur** `run_disturbance_observer()` | appelé, sorties inutilisées | Calcule `theta_e_hat`, `omega_e_hat`, `f_hat` à 10 kHz, mais la commande utilise `angle_filtered`/`theta_e_encoder` et `omega_m`. Consomme du CPU sans effet. |
| `control_torque_imc()` | jamais appelé | Commande alternative (modèle interne, gains `cK`) définie mais non utilisée. |
| `theta_m_ref_l`, `MY_INVERTER_ID_L` | commentés | Ancien essai de gestion de 2 moteurs sur une seule carte. |
| Champ `mode` + watchdog interrupteur | commentés | Préparation du sélecteur IDLE/POWER 3 positions, non activé. |

> **/!\ Réinitialisation d'unwrap manquante.** `init_filt_and_reg()` remet la PLL à 0
> (`pllangle.reset(0.F)`) mais **pas** `angle_prev`, `angle_elec_unwrapped` ni `int_pos`. À chaque
> arrêt PWM (`stop_pwm_and_reset_states_ifnot`), `angle_prev` garde son ancienne valeur pendant que
> la PLL saute à 0 → correction d'unwrap parasite → **saut de `theta_m`** à chaque cycle
> POWER→IDLE→POWER. Correctif : ajouter dans `init_filt_and_reg()` :
> ```c
> angle_prev = 0.0F;  angle_elec_unwrapped = 0.0F;  int_pos = 0.0F;
> ```

---

## 11. Utilisation

### Démarrage

1. Vérifier `MY_INVERTER_ID` (**5** = gauche, **4** = droite) et recompiler pour chaque carte.
2. Flasher.
3. Mettre le bus DC sous tension.
4. Calibration des offsets (~0,2 s, LED allumée puis éteinte), puis **armement automatique**.
5. Ouvrir un terminal série pour la télémétrie.

### Touches du terminal

| Touche | Action |
|---|---|
| `p` | Demande le mode POWER + relance ScopeMimicry |
| `i` | Demande le mode IDLE - **c'est l'arrêt** |
| `c` | **Bascule Hall ↔ encodeur** (`use_encoder`). À faire en IDLE uniquement. |
| `u` / `d` | `theta_m_ref` ± 0,5 rad (test manuel de consigne, **actif**) |
| `s` / `x` | `K_posi` ± 0,01 (gain intégral) |
| `q` / `w` | `K_posp` ± 0,01 (gain proportionnel) |
| `z` / `e` | `K_posd` ± 0,0001 (gain dérivé) |
| `t` | Affiche la valeur brute du compteur d'encodeur |
| `r` | Télécharge le buffer ScopeMimicry (`dump_scope_datas`) |
| `m` | Bascule l'affichage : télémétrie ↔ dump continu du scope (pour OwnPlot) |
| `a` | Relance une acquisition ScopeMimicry |

> Les touches `u`/`d` écrivent directement `theta_m_ref` - la trame RS485 suivante l'écrasera.
> Utiles pour tester sans IHM.

### Lecture de la télémétrie

Ligne toutes les 250 ms, champs séparés par `:` :

```
K_posi : K_posp : K_posd : theta_m_ref : theta_m : V_high : hall_angle :
Iq_max : manual_Iq_ref : i_alpha_ref_raw : i_alpha_ref_filtered : Idq.q :
control_state : angle_index
```

**Diagnostic rapide :**

| Symptôme | Interprétation |
|---|---|
| `control_state = 3` | Surintensité → `ERROR_ST`. Appuyer sur `i` pour réarmer. |
| `theta_m_ref` figé alors que le joystick bouge | Plus de trames valides reçues (bus, checksum, désalignement). |
| Terminal figé | Firmware bloqué — pas un problème de bus. |
| LED qui bascule | Trame **valide et adressée à cette carte** reçue (`spin.led.toggle()`). Témoin direct de la santé du bus. |

### ScopeMimicry

Déclaré **7 voies** : `ScopeMimicry scope(SCOPE_SIZE, 7)`. Voies actives (cohérentes avec le 7) :

```
Vq · Vd · Iq_meas · Id_meas · theta_m_ref · theta_m · angle_filtered
```

Décimation 10 → un point toutes les **1 ms** ; `SCOPE_SIZE = 3000` → **3 s**.
Déclenchement `mytrigger()` : `control_state == POWER_ST`.

> **/!\ Deux règles ScopeMimicry :**
> 1. Le nombre de `connectChannel` **actifs** doit égaler le second argument (**7**). Pour ajouter
>    une voie (ex. `Iq_ref` pour voir la saturation de couple), en retirer une autre ou augmenter le 7.
> 2. Le mode `memory_print` (touche `'m'`) imprime **10 indices (0–9)** alors qu'il n'existe que 7
>    voies. Avec `'m'`/OwnPlot, réduire les `printk` aux indices 0–6. Avec `'r'`, pas de problème.

---

## 12. Retour en fonctionnement normal (non-test)

```c
// init_variables()
asked_mode = IDLEMODE;        // au lieu de POWERMODE

// application_task(), fin de OFFSET_ST
control_state = IDLE_ST;      // au lieu de POWER_ST

// IDLE_ST, avant control_state = POWER_ST
theta_m_ref = theta_m;        // démarrage sans à-coup (recommandé)
```

Ces trois lignes rétablissent l'armement manuel par `'p'` et la vérification de la tension de bus.

---

## 13. Paramètres de réglage

| Paramètre | Valeur | Rôle |
|---|---|---|
| `Ts` | 100 µs | Période de la boucle critique (10 kHz) |
| `K_posi / K_posp / K_posd` | 15.13 / 3.72 / 0.208 | Boucle de position (réglables à chaud) |
| `Kp` / `Ti` | 1.4 / 0.002029 | PID de courant d et q |
| `Iq_max` | 5.0 A | Couple maximal |
| `k_angle_offset` | π | Calage électrique Hall ↔ bobinages |
| `encoder_offset` | −0.1 (à calibrer) | Calage électrique encodeur ↔ bobinages |
| `ENCODER_COUNTS_PER_REV` | 133 (à vérifier) | Comptes encodeur par tour moteur |
| `sector[]` | `{-1,5,1,0,3,4,2}` | Table Hall → secteur (dépend du câblage moteur) |
| `pole_pairs` | 4 | Moteur 8 pôles |
| `V_HIGH_MIN` | 5 V | Seuil d'armement (contourné en mode test) |
| `AC/DC_CURRENT_LIMIT` | 13 A | Seuils de surintensité |

Moteur qui vibre, force sans tourner, ou tourne à l'envers : commencer par `k_angle_offset` et la
table `sector[]` (calage électrique). En mode encodeur, commencer par `encoder_offset`.
