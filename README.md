# Commande FOC — Carte OwnVerter (moteur gauche, ID 5)

Firmware de contrôle de position d'un moteur synchrone (PMSM) par commande vectorielle
(Field Oriented Control), sur carte **OwnTech OwnVerter**. La consigne de position arrive
par un bus **RS485** depuis une IHM (Raspberry Pi + joystick).

> Ce fichier documente le code tel qu'il est écrit. Les points marqués **/!\** signalent des
> comportements réels du code qui peuvent surprendre, ou du code présent mais inactif.

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
                                             capteurs Hall ← position
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
| Encodeur | initialisé sur `TIMER3` — **/!\ non utilisé dans la commande** (§ 9) |
| Communication | RS485, 115200 bauds, trames de 6 octets |
| Bus DC | seuil minimal d'armement `V_HIGH_MIN = 5.0 V` |

### Mécanique (constantes du code)

```c
capstan_radius = 1.5 cm     capstan_gear = 45     motor_gear = 15
line_cm_to_motor_rad = capstan_gear / (capstan_radius * motor_gear) = 2.0
```

→ **1 cm de fil = 2 rad à l'axe moteur.** C'est le facteur unique de conversion entre la
longueur de fil envoyée par l'IHM et la consigne angulaire interne.

---

## 3. Architecture logicielle - 3 tâches

| Tâche | Période | Rôle |
|---|---|---|
| `loop_critical_task()` | **100 µs (10 kHz)** | Mesures, position, observateur, FOC, rapports cycliques |
| `application_task()` | 250 ms | Affichage terminal, machine d'états, handshake RS485 |
| `loop_background_task()` | bloquante | Lecture des touches clavier (`console_getchar()`) |
| `rs485_rx_callback()` | sur réception | Décodage d'une trame RS485 → `theta_m_ref` |

Toutes sont créées dans `setup_routine()`, qui initialise aussi la puissance, les capteurs,
le RS485, les GPIO Hall et ScopeMimicry.

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

### Adressage

```c
#define MY_INVERTER_ID 5   // LEFT_MOTOR = 5, RIGHT_MOTOR = 4
```

Sur un bus RS485, **toutes** les cartes reçoivent **toutes** les trames. C'est le test
`payload->target_id == MY_INVERTER_ID` qui fait qu'une carte n'applique que la trame qui lui
est destinée. Pour la deuxième carte : recompiler avec `MY_INVERTER_ID 4`.

### Checksum

```c
static uint8_t compute_checksum(const uint8_t* data, int len) {
    uint8_t cs = 0;
    for (int i = 0; i < len; i++) cs ^= data[i];
    return cs;
}
```

En réception, une trame dont le XOR ne correspond pas est **rejetée** :

```c
if (payload->checksum != expected) return;   // theta_m_ref garde sa valeur précédente
```

**Conséquence :** en cas de corruption, la consigne se **fige** au lieu de sauter à une valeur
fausse. Le checksum **détecte** la corruption, il ne la **corrige pas**.

> **Contrat obligatoire :** l'IHM doit envoyer exactement 6 octets avec le même XOR. Si un seul
> des deux côtés change de taille de trame, plus rien ne passe (les consignes restent à 0).

### Handshake

Tant que `handshake_complete == false`, la carte réémet le code 111 toutes les 250 ms depuis
`application_task()`. Le drapeau passe à `true` à la **première trame valide** reçue.

---

## 5. Mesure de position et de vitesse - `get_position_and_speed()`

Chaîne complète, exécutée à 10 kHz :

1. **Lecture des 3 Hall** → index de secteur
   `angle_index = HALL1 + 2*HALL2 + 4*HALL3` (valeurs 1 à 6 valides ; 0 et 7 impossibles)
2. **Table de secteurs** → angle électrique brut, par pas de 60° :
   ```c
   sector[] = {-1, 5, 1, 0, 3, 4, 2};
   hall_angle = ot_modulo_2pi(PI/3 * sector[angle_index] + k_angle_offset);
   ```
   `k_angle_offset = PI` est le **calage électrique** entre les Hall et les bobinages.
3. **PLL** (`pllangle`) → lissage en angle continu `angle_filtered` (les Hall seuls ne donnent
   qu'un angle en escalier de 60°).
4. **Déroulement (unwrap)** → `angle_elec_unwrapped`, angle électrique cumulé sans saut à ±π.
5. **Angle mécanique** : `theta_m = angle_elec_unwrapped / pole_pairs`
6. **Vitesse** : `omega_m = filtre_passe_bas(pllDatas.w) / pole_pairs`

`theta_m` est donc un angle **cumulé** (multi-tours), ce qui est indispensable pour asservir une
longueur de fil.

---

## 6. Boucle de commande - `control_torque()`

### Étage 1 : position → couple

```c
angle_4_control = angle_filtered;              // angle utilisé pour Park/Clarke
float32_t pos_error = theta_m - theta_m_ref;
int_pos += pos_error * Ts * anti_windup;       // intégrateur

Idq_ref.q = -K_posi*int_pos - K_posp*theta_m - K_posd*omega_m;
```

Retour d'état avec action intégrale. Gains par défaut :

| Gain | Variable | Valeur | Agit sur |
|---|---|---|---|
| `K_posi` | intégral | 15.13 | erreur de position intégrée |
| `K_posp` | proportionnel | 3.72 | position `theta_m` |
| `K_posd` | dérivé | 0.208 | vitesse `omega_m` |

**/!\ À noter :** seul le terme intégral voit `theta_m_ref`. Les termes P et D agissent sur
`theta_m` et `omega_m` **absolus**, pas sur l'erreur. Le suivi de consigne en régime établi
repose donc entièrement sur l'intégrateur - c'est voulu dans une commande par retour d'état,
mais cela explique que la réponse dépende fortement de `K_posi`.

**Anti-windup :** si `Idq_ref.q` sature à `±Iq_max` (5.0 A), `anti_windup = 0` gèle
l'intégrateur pour éviter son emballement.

### Étage 2 : FOC (courant)

```c
Iabc.a = I1_low_value;  Iabc.b = I2_low_value;  Iabc.c = -(Iabc.a + Iabc.b);
Idq = Transform::to_dqo(Iabc, angle_4_control);      // Park : abc → dq

Vdq.d = pi_d.calculateWithReturn(Idq_ref.d, Idq.d);  // Idq_ref.d = 0 (pas d'affaiblissement)
Vdq.q = pi_q.calculateWithReturn(Idq_ref.q, Idq.q);

Vabc = Transform::to_threephase(Vdq, angle_4_control); // Park inverse : dq → abc
```

PID identiques sur d et q : `Kp = 1.4`, `Ti = 0.002029`, bornes **±12 V**
(= `MIN_DC_VOLTAGE × 0.4`, marge de modulation).

### Étage 3 : rapports cycliques

```c
duty_abc.x = Vabc.x / V_high_filtered + 0.5;   // 0.5 = point milieu
```

Puis `apply_duties()` → `LEG1/2/3`.

---

## 7. Machine d'états

```
        OFFSET_ST ──(après 2000 cycles = 0.2 s)──▶ POWER_ST   /!\ voir ci-dessous
            │                                        │
        PWM arrêté                              touche 'i'
                                                     ▼
                                                  IDLE_ST ──('p' + V_high > 5 V)──▶ POWER_ST
                                                     
        ERROR_ST ◀── surintensité (n'importe quel état) ──▶ 'i' ──▶ IDLE_ST
```

| État | Valeur | Comportement |
|---|---|---|
| `OFFSET_ST` | 0 | Calibration de l'offset des capteurs de courant (moyenne sur 2000 cycles). PWM arrêté. |
| `IDLE_ST` | 1 | Au repos, PWM arrêté. Armement si `asked_mode == POWERMODE` **et** `V_high_filtered > 5 V`. |
| `POWER_ST` | 2 | Commande active, PWM en marche. |
| `ERROR_ST` | 3 | Surintensité détectée. PWM arrêté. Sortie uniquement par `'i'`. |

**/!\ Configuration actuelle (mode test) :** deux modifications court-circuitent l'armement manuel :

```c
asked_mode = POWERMODE;      // dans init_variables() — au lieu de IDLEMODE
control_state = POWER_ST;    // fin de OFFSET_ST — au lieu de IDLE_ST
```

→ **La carte s'arme automatiquement ~0,2 s après le flash**, sans appuyer sur `'p'`.
La calibration d'offset est bien effectuée avant. En revanche, ce chemin **ne vérifie pas
`V_high_filtered`** : le test de tension n'existe que dans la transition `IDLE_ST → POWER_ST`,
qui est contournée. À remettre en `IDLE_ST` pour un fonctionnement normal.

**/!\ Au démarrage, `theta_m_ref` vaut 0** alors que `theta_m` vaut la position réelle : le moteur
part immédiatement rejoindre la position zéro. Pour démarrer sans à-coup, décommenter dans
`IDLE_ST` la ligne `theta_m_ref = theta_m;`.

---

## 8. Sécurités

| Protection | Seuil | Mécanisme |
|---|---|---|
| Surintensité AC (I1, I2) | ±13 A | `error_counter++` |
| Surintensité DC (I_high) | 13 A | `error_counter++` |
| Filtrage des faux positifs | - | `error_counter--` tous les 1000 cycles ; bascule en `ERROR_ST` au-delà de 3 |
| Saturation de couple | `Iq_max = 5.0 A` | Écrêtage de `Idq_ref.q` + anti-windup |
| Saturation de tension | ±12 V | Bornes des PID d/q |
| Trame corrompue | XOR | Trame rejetée, consigne figée |

> **Il n'existe aucun arrêt d'urgence logiciel indépendant du terminal.** Le seul arrêt est la
> touche `'i'`. Pour une utilisation hors banc, prévoir un **arrêt d'urgence matériel** qui coupe
> la puissance sans dépendre du logiciel ni du bus RS485.

---

## 9. Code présent mais **inactif** /!\

Ces blocs sont compilés (ou commentés) mais n'influencent pas le moteur. Utile à savoir avant
de modifier quoi que ce soit.

| Élément | État | Détail |
|---|---|---|
| **Encodeur** | initialisé, non lu | `startLogIncrementalEncoder(TIMER3)` est actif, mais `encoder_count = ...` est **commenté** dans `get_position_and_speed()`. Seule la touche `'t'` lit le compteur. `theta_m_encoder`, `theta_e_encoder`, `encoder_offset` restent inutilisés. |
| `ENCODER_COUNTS_PER_REV` | valeur tronquée | `100*4/(45.0/15.0)` = 133,33 → tronqué à **133** (`int32_t`). Erreur systématique de 0,25 % par tour. Mélange aussi résolution encodeur et rapport d'engrenage : à revoir selon l'emplacement réel de l'encodeur (axe moteur ou capstan). |
| **Observateur** `run_disturbance_observer()` | appelé, sorties inutilisées | Calcule `theta_e_hat`, `omega_e_hat`, `f_hat` à 10 kHz, mais `control_torque()` utilise `angle_filtered` (PLL) et `omega_m` (filtre). Consomme du CPU sans effet. |
| `control_torque_imc()` | jamais appelé | Commande alternative (modèle interne, gains `cK`) définie mais non utilisée. |
| `theta_m_ref_l`, `MY_INVERTER_ID_L` | supprimés/commentés | Ancien essai de gestion de 2 moteurs sur une seule carte. |
| Checksum de la réponse | fonctionne par coïncidence | `compute_checksum(..., sizeof(InverterPayload_t) - 1)` est utilisé pour `InverterReply_t`. Les deux structures font 6 octets, donc le résultat est correct **aujourd'hui** ; si l'une change de taille, le bug apparaît. Préférer `sizeof(InverterReply_t) - 1`. |

---

## 10. Utilisation

### Démarrage

1. Vérifier `MY_INVERTER_ID` (**5** = gauche, **4** = droite) et recompiler pour chaque carte.
2. Flasher.
3. Mettre le bus DC sous tension.
4. La carte calibre les offsets (~0,2 s, LED allumée puis éteinte) puis **s'arme automatiquement**.
5. Ouvrir un terminal série pour voir la télémétrie (voir plus bas).

### Touches du terminal

| Touche | Action |
|---|---|
| `p` | Demande le mode POWER (`asked_mode = POWERMODE`) + relance ScopeMimicry |
| `i` | Demande le mode IDLE — **c'est l'arrêt** |
| `u` / `d` | `theta_m_ref` ± 0,5 rad (test manuel de consigne) |
| `s` / `x` | `K_posi` ± 0,01 (gain intégral) |
| `q` / `w` | `K_posp` ± 0,01 (gain proportionnel) |
| `z` / `e` | `K_posd` ± 0,0001 (gain dérivé) |
| `t` | Affiche la valeur brute du compteur d'encodeur |
| `r` | Télécharge le buffer ScopeMimicry (`dump_scope_datas`) |
| `m` | Bascule l'affichage : télémétrie ↔ dump continu du scope (pour OwnPlot) |
| `a` | Relance une acquisition ScopeMimicry |

> Les touches `u`/`d` écrivent directement `theta_m_ref` — la trame RS485 suivante l'écrasera.
> Elles servent au test sans IHM.

### Lecture de la télémétrie

Ligne affichée toutes les 250 ms, champs séparés par `:` :

```
K_posi : K_posp : K_posd : theta_m_ref : theta_m : V_high : hall_angle :
Iq_max : manual_Iq_ref : i_alpha_ref_raw : i_alpha_ref_filtered : Idq.q :
control_state : angle_index
```

**Diagnostic rapide :**

| Symptôme | Interprétation |
|---|---|
| `control_state = 3` | Surintensité → `ERROR_ST`. Appuyer sur `i` pour réarmer. |
| `theta_m_ref` figé alors que le joystick bouge | Plus de trames valides reçues (bus, checksum, ou désalignement). |
| Terminal figé | Firmware bloqué - ce n'est pas un problème de bus. |
| LED qui bascule | Une trame **valide et adressée à cette carte** vient d'être reçue (`spin.led.toggle()` dans le callback). Témoin le plus direct de la santé du bus. |

### ScopeMimicry

6 voies enregistrées, décimation 10 → un point toutes les **1 ms**, sur 3000 points = **3 s**.

Voies actives : `Vq`, `I1_low_value`, `I2_low_value`, `hall_angle`, `theta_m_ref`, `omega_m`.

Déclenchement : `mytrigger()` renvoie vrai quand `control_state == POWER_ST`.

> Le nombre de `scope.connectChannel(...)` **actifs** doit rester égal au second argument de
> `ScopeMimicry scope(SCOPE_SIZE, 6)`. Si vous décommentez une voie, décommentez-en une autre
> ou augmentez ce nombre.

---

## 11. Paramètres de réglage

| Paramètre | Valeur | Rôle |
|---|---|---|
| `Ts` | 100 µs | Période de la boucle critique (10 kHz) |
| `K_posi / K_posp / K_posd` | 15.13 / 3.72 / 0.208 | Boucle de position (réglables à chaud) |
| `Kp` / `Ti` | 1.4 / 0.002029 | PID de courant d et q |
| `Iq_max` | 5.0 A | Couple maximal |
| `k_angle_offset` | π | Calage électrique Hall ↔ bobinages |
| `sector[]` | `{-1,5,1,0,3,4,2}` | Table Hall → secteur (dépend du câblage moteur) |
| `pole_pairs` | 4 | Moteur 8 pôles |
| `V_HIGH_MIN` | 5 V | Seuil d'armement (contourné en mode test) |
| `AC/DC_CURRENT_LIMIT` | 13 A | Seuils de surintensité |

Si le moteur vibre, force sans tourner, ou tourne à l'envers : commencer par `k_angle_offset`
et la table `sector[]`, qui définissent le calage électrique.

---

## 12. Pour revenir en fonctionnement normal (non-test)

```c
// init_variables()
asked_mode = IDLEMODE;        // au lieu de POWERMODE

// application_task(), fin de OFFSET_ST
control_state = IDLE_ST;      // au lieu de POWER_ST

// IDLE_ST, avant control_state = POWER_ST
theta_m_ref = theta_m;        // démarrage sans à-coup (optionnel mais recommandé)
```

Ces trois lignes rétablissent l'armement manuel par `'p'` **et** la vérification de la tension
de bus.
