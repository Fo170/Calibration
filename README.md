# Calibration — calibration linéaire générique (offset + gain)

Bibliothèque **header-only** de calibration linéaire à deux coefficients
(`offset`, `gain`), réutilisable sur ESP8266, ESP32, Arduino (AVR, SAM, STM32…)
et PC.

Elle **dépend de [Smoothing](https://github.com/Fo170/Smoothing.git)** pour :
- le **lissage du brut en entrée** (`push()` + `applySmoothed()`) ;
- la **régression robuste** (`solvePointsRobust()`, rejet des points aberrants).

Auteur : Olivier FOURNET (olivier.fournet@free.fr) — org GitHub Fo170  •  Licence : GPL-3.0-only  •  Version : 1.1.0

---

## Sommaire

- [Les deux formes de correction](#les-deux-formes-de-correction)
- [Installation](#installation)
- [Utilisation rapide](#utilisation-rapide)
- [API détaillée](#api-détaillée)
- [Modes de résolution](#modes-de-résolution)
- [Lissage du brut (Smoothing)](#lissage-du-brut-smoothing)
- [Régression robuste](#régression-robuste)
- [Sémantique des solveurs (composition)](#sémantique-des-solveurs-composition)
- [Intégration dans un projet](#intégration-dans-un-projet)
- [Exemples](#exemples)
- [Limites et pièges](#limites-et-pièges)
- [Licence](#licence)

---

## Les deux formes de correction

| Forme | Formule | Usage typique |
|-------|---------|---------------|
| `OFFSET_BEFORE_GAIN` (défaut) | `y = (x + offset) * gain` | capteurs température / humidité / pression |
| `OFFSET_AFTER_GAIN` | `y = x * gain + offset` | variante « offset après gain » |

`offset` corrige le zéro, `gain` corrige l'échelle. `unapply(y)` fait l'inverse
(diagnostic).

> ⚠️ **Règle d'or** : lisser le **BRUT** puis appliquer la **calibration**.
> Ne jamais moyenner des valeurs déjà calibrées : un changement de coefficients
> fausserait la moyenne.

---

## Installation

### PlatformIO (recommandé — depuis GitHub)

`Calibration` dépend de `Smoothing`. Déclarez **les deux** dans le `platformio.ini`
du projet consommateur (un exemple est fourni dans `platformio.ini`) :

```ini
[env:esp32dev]
platform  = espressif32
board     = esp32dev
framework = arduino
lib_deps  =
    https://github.com/Fo170/Calibration.git
    https://github.com/Fo170/Smoothing.git
```

> ⚠️ Utilisez toujours l'**URL git complète** (`https://github.com/Fo170/Calibration.git`),
> jamais la forme courte `Fo170/Calibration` (non publiée sur le registre PlatformIO).
> Pour **figer une version**, ajoutez un tag : `...#v1.1.0` / `...#v1.1.1`.

Puis dans le code :

```cpp
#include <Calibration.h>   // inclut <Smoothing.h>
```

### Arduino IDE

Copiez les dossiers `Calibration/` **et** `Smoothing/` dans
`libraries/` (chacun avec `src/`).

### Installation locale (développement / dépannage)

Placez `Calibration/` **et** `Smoothing/` dans le dossier `lib/` du projet.

---

## Utilisation rapide

```cpp
#include <Calibration.h>

Calibration cal;                       // offset=0, gain=1 (neutre)
                                       // un Calibration PAR voie de mesure

void setup() {
  // Exemple : étalonnage sur 2 points connus (valeurs MESURÉES = sorties actuelles)
  float mes[] = { 10.0f, 90.0f };      // valeurs lues (brutes ou déjà calibrées)
  float vus[] = { 12.0f, 88.0f };      // valeurs de référence
  cal.calibrate(mes, vus, 2, true);    // calcule offset + gain et les applique
}

void loop() {
  float brut = lireCapteur();

  cal.push(brut);                      // 1 appel par acquisition (lissage Smoothing)
  float y = cal.applySmoothed(Smoother::EMA, 8);  // lissé PUIS calibré

  // variante : lisser et calibrer en deux temps
  // float lisse = cal.smooth(Smoother::EMA, 8);
  // float y     = cal.apply(lisse);

  delay(2000);
}
```

`apply()` reste **stateless** (calibration d'une valeur directe) ; `push()` +
`applySmoothed()` ajoutent le lissage à état.

---

## API détaillée

### La classe `Calibration`

```cpp
class Calibration {
  public:
    enum Form { OFFSET_BEFORE_GAIN = 0, OFFSET_AFTER_GAIN = 1 };
    enum Mode { MODE_DIRECT=0, MODE_ONE_POINT=1, MODE_TWO_POINTS=2,
                MODE_REGRESSION=3, MODE_ZERO_SPAN=4, MODE_RESET=5 };

    static const int MAX_POINTS = 32;   // limite de solvePointsRobust

    Calibration(float offset = 0.0f, float gain = 1.0f,
                Form form = OFFSET_BEFORE_GAIN);

    // Coefficients
    void  set(float offset, float gain);
    void  setOffset(float o);
    void  setGain(float g);
    void  setForm(Form f);
    float offset() const;
    float gain() const;
    Form  form() const;
    void  reset();                       // offset=0, gain=1 (+ reset du lissage)

    // Application
    float apply(float raw) const;
    float unapply(float y) const;        // NAN si gain == 0

    // Lissage du brut (dépend de Smoothing)
    void  push(float raw);
    float smooth(uint8_t type, uint8_t win, float k = 3.0f, float trim = 0.15f) const;
    float applySmoothed(uint8_t type, uint8_t win, float k = 3.0f, float trim = 0.15f) const;
    Smoother&       smoother();
    const Smoother& smoother() const;

    // Résolution
    bool solveOnePoint(float m, float d, float& outOffset, float& outGain) const;
    bool solvePoints(const float* measured, const float* desired, int n,
                     bool solveGain, float& outOffset, float& outGain) const;
    bool solvePointsRobust(const float* measured, const float* desired, int n,
                           bool solveGain, float& outOffset, float& outGain,
                           uint8_t filter = Smoother::HAMPEL,
                           uint8_t win = 8, float k = 3.0f) const;
    bool calibrate(const float* measured, const float* desired, int n, bool solveGain);
};
```

| Méthode | Rôle |
|---------|------|
| `apply(x)` | Calcule `y` à partir de `x` selon la `Form`. **Sans état.** |
| `unapply(y)` | Inverse la correction (`NAN` si `gain == 0`). |
| `push(v)` | Mémorise une mesure **brute** dans le `Smoother` interne. 1 appel/acquisition. |
| `smooth(type, win, k, trim)` | Dernière valeur **lissée** (brute), sans calibration. |
| `applySmoothed(type, win, k, trim)` | `apply(smooth(...))` : lissé **puis** calibré. |
| `smoother()` | Accès direct au `Smoother` interne (réglages avancés, `setKalman`, `reset`). |
| `set/setOffset/setGain/setForm` | Écriture des coefficients / de la forme. |
| `reset()` | `offset=0`, `gain=1` **et** `_smoother.reset()` (la `Form` est conservée). |
| `solveOnePoint` | 1 couple → ajuste l'**offset**, **conserve le gain**. |
| `solvePoints` | N couples → offset (+ gain si `solveGain`), moindres carrés. |
| `solvePointsRobust` | Comme `solvePoints`, mais filtre les **points aberrants** via Smoothing. |
| `calibrate` | `solvePoints` **puis** `set()` (raccourci résoudre + appliquer). |
| `modeName(m)` | Libellé français d'un `Mode` (UI/logs) — le mode est **documentaire**, aucun solveur ne le prend en paramètre. |

---

## Modes de résolution

| Mode | Entrée | Calcule |
|------|--------|---------|
| `MODE_DIRECT` | offset + gain | rien (saisie directe) |
| `MODE_ONE_POINT` | 1 couple | offset seul |
| `MODE_TWO_POINTS` | 2 couples | offset + gain |
| `MODE_REGRESSION` | 2..N couples | offset + gain (moindres carrés) |
| `MODE_ZERO_SPAN` | 2 couples (bas/haut) | offset + gain |
| `MODE_RESET` | — | offset = 0, gain = 1 |

L'`enum Mode` est **documentaire** : il identifie la méthode pour l'UI/les logs.
Les valeurs `0..5` sont **figées** (configurations persistées) — ne pas les
renuméroter ; ajouter à la suite.

---

## Lissage du brut (Smoothing)

`Calibration` embarque un `Smoother` (voir la doc de
[Smoothing](https://github.com/Fo170/Smoothing.git) pour les 19 méthodes) :

```cpp
cal.push(brut);                                  // à chaque acquisition
float y = cal.applySmoothed(Smoother::EMA, 8);   // lissé + calibré
```

- Le lissage porte **toujours sur le brut**, jamais sur du calibré.
- `win` (fenêtre) et `k` / `trim` ont la même sémantique que dans `Smoothing`.
- `reset()` réinitialise aussi l'état de lissage.
- Pour régler Kalman : `cal.smoother().setKalman(q, r);`

---

## Régression robuste

`solvePointsRobust()` **écarte** les points aberrants avant la régression, en
filtrant `measured` et `desired` via un `Smoother` local (`push()` + `value()`) :
un point que le filtre a remplacé (donc jugé aberrant) est **exclu** du calcul
(il n'est pas remplacé par la médiane).

```cpp
float mes[5] = { 0, 25, 50, 75, 100 };
float vus[5] = { 0, 26, 49, 200, 101 };          // 200 = point aberrant
float o, g;
cal.solvePointsRobust(mes, vus, 5, true, o, g,
                      Smoother::HAMPEL, 8, 3.0f); // filtre Hampel (défaut)
```

- `filter` : filtre « conserve la valeur si elle n'est pas aberrante »
  (`Smoother::HAMPEL` par défaut). ⚠️ Un filtre qui renvoie toujours une moyenne
  (ex. `ZSCORE`) écarterait **tous** les points → à éviter ici.
- `win` : fenêtre de détection (auto-réduite au démarrage).
- `k` : seuil (nb de MAD pour `HAMPEL`).
- **Pas d'allocation dynamique** : limité à `MAX_POINTS` (32) ; au-delà → `false`.
- Si tous les points sont écartés → `false`.

---

## Sémantique des solveurs (composition)

Les solveurs **composent** la nouvelle correction **par-dessus** les coefficients
courants. Les valeurs `measured` sont donc les **sorties actuelles** de l'objet
(`apply(brut)`), et `desired` les cibles.

Avec les coefficients par défaut (`offset = 0`, `gain = 1`), cela revient à
traiter `measured` = valeur brute. Pour repartir du brut sur un objet déjà
calibré : `cal.reset()` (ou un objet neuf).

---

## Intégration dans un projet

```
   capteur ──► valeur BRUTE ──► Calibration::push()
                                        │
                                        ▼
                       Calibration::applySmoothed()  ──► valeur affichée
                       ( = apply(smooth()) )
```

- Un `Calibration` **par voie** (température, humidité, pression, …).
- `push()` à chaque acquisition ; `applySmoothed()` au moment de publier.
- La calibration s'applique **après** le lissage.

---

## Exemples

### Étalonnage sur 2 points (zéro / span)

```cpp
Calibration cal;

float mes[] = { 10.0f, 90.0f };
float vus[] = { 12.0f, 88.0f };
cal.calibrate(mes, vus, 2, true);   // offset + gain

float y = cal.apply(50.0f);
```

### Ajustement d'offset seul (1 point)

```cpp
float o, g;
cal.solveOnePoint(/*mesuré*/ 25.0f, /*voulu*/ 25.4f, o, g);
cal.set(o, g);                       // g = gain courant (conservé)
```

### Saisie directe

```cpp
cal.setForm(Calibration::OFFSET_AFTER_GAIN);
cal.set(1.5f, 0.98f);                // offset = 1.5, gain = 0.98
```

### Chaîne complète (brut → lissage → calibration)

```cpp
cal.push(lireCapteur());
float y = cal.applySmoothed(Smoother::HAMPEL, 8, 3.0f);
```

### Régression robuste N points

```cpp
float mes[5] = { 0, 25, 50, 75, 100 };
float vus[5] = { 0, 26, 49, 200, 101 };   // 200 = point aberrant
float o, g;
if (cal.solvePointsRobust(mes, vus, 5, true, o, g)) {
  cal.set(o, g);
}
```

---

## Limites et pièges

- **`measured` = sortie actuelle, pas forcément le brut** : les solveurs composent
  sur `_offset`/`_gain` existants. Sur un objet déjà calibré, faire `reset()`
  avant de passer des valeurs brutes.
- **`gain == 0`** : `unapply` renvoie `NAN` ; `solveOnePoint`/`solvePoints` renvoient
  `false`.
- **Points dégénérés** : deux points identiques (pente indéfinie) → `false`.
- **`solvePoints` avec `solveGain == false`** : retombe sur un solveur 1 point
  (`m[0]`, `d[0]` uniquement).
- **`reset()` ne touche pas `_form`** : rappeler `setForm()` si on change de forme.
- **Ne pas renuméroter les enums** `Form` et `Mode` (configurations persistées).
- **`solvePointsRobust`** est limité à `MAX_POINTS` (32) et dépend de Smoothing.
- **Pile sur AVR** : `solvePointsRobust` utilise 2 `Smoother` locaux + 2×`float[32]`
  (≈ 600 o de pile au pic). Sur cible AVR (≈ 2 Ko de SRAM), réduire `MAX_POINTS`
  ou éviter l'appel en contexte de pile déjà chargée.
- **Dépendance Smoothing obligatoire** : `#include <Smoothing.h>` doit être
  résolu (via `lib_deps` / dossier `libraries/` / `lib/`).

---

## Licence

GPL-3.0-only. Header-only. Dépend de `Smoothing` (GPL-3.0).
