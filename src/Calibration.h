/*
  Calibration.h - Bibliothèque générique de calibration linéaire (header-only)
  ------------------------------------------------------------------------------
  Version : 1.1.0

  Modélise une correction linéaire à deux coefficients (offset, gain) selon
  deux formes :
    - OFFSET_BEFORE_GAIN : y = (x + offset) * gain   (capteurs T°/Hum/Pression...)
    - OFFSET_AFTER_GAIN  : y = x * gain + offset      (variante : offset après gain)

  Fournit plusieurs modes de calcul à partir de couples (mesuré, voulu) :
    - point unique   : offset seul (gain conservé)
    - deux points    : offset + gain
    - régression N   : moindres carrés (offset + gain), robuste au bruit
    - zéro/span      : identique à deux points (vocabulaire métrologie)
    - réinitialiser  : offset = 0, gain = 1

  DÉPENDANCE : bibliothèque Smoothing (https://github.com/Fo170/Smoothing.git)
    - lissage du brut en entrée : push() + applySmoothed() ;
    - régression robuste : solvePointsRobust() rejette les points aberrants.

  Licence : GPL-3.0-only
*/

#ifndef CALIBRATION_H
#define CALIBRATION_H


// Version de la librairie (source UNIQUE — agrégée par liste_lib.h)
#define __CALIBRATION_H_VERSION__ "1.1.0"
#include <math.h>
#include <Smoothing.h>

class Calibration {
  public:
    enum Form { OFFSET_BEFORE_GAIN = 0,   // y = (x + offset) * gain
                OFFSET_AFTER_GAIN  = 1 }; // y = x * gain + offset

    enum Mode {
      MODE_DIRECT      = 0,  // saisie directe offset + gain
      MODE_ONE_POINT   = 1,  // offset seul
      MODE_TWO_POINTS  = 2,  // offset + gain
      MODE_REGRESSION  = 3,  // offset + gain (moindres carrés, 2..N)
      MODE_ZERO_SPAN   = 4,  // offset + gain (bas/haut)
      MODE_RESET       = 5   // offset = 0, gain = 1
    };

    // Nombre max de points traités par solvePointsRobust (pas d'allocation dynamique).
    static const int MAX_POINTS = 32;

    // Libellé d'un mode de résolution (interface web / logs).
    static const char* modeName(uint8_t m) {
      switch (m) {
        case MODE_DIRECT:     return "Saisie directe";
        case MODE_ONE_POINT:  return "Point unique";
        case MODE_TWO_POINTS: return "Deux points";
        case MODE_REGRESSION: return "Regression N points";
        case MODE_ZERO_SPAN:  return "Zero/span";
        case MODE_RESET:      return "Reinitialiser";
      }
      return "?";
    }

    Calibration(float offset = 0.0f, float gain = 1.0f, Form form = OFFSET_BEFORE_GAIN)
      : _offset(offset), _gain(gain), _form(form) {}

    // --- Coefficients ---
    void  set(float offset, float gain) { _offset = offset; _gain = gain; }
    void  setOffset(float o) { _offset = o; }
    void  setGain(float g)   { _gain = g; }
    void  setForm(Form f)    { _form = f; }
    float offset() const     { return _offset; }
    float gain() const       { return _gain; }
    Form  form() const       { return _form; }

    // Réinitialise les coefficients (la Form n'est pas modifiée) et le lissage.
    void reset() { _offset = 0.0f; _gain = 1.0f; _smoother.reset(); }

    // --- Application ---
    float apply(float raw) const {
      return (_form == OFFSET_BEFORE_GAIN) ? (raw + _offset) * _gain
                                           : raw * _gain + _offset;
    }

    // Retour vers la valeur non calibrée (diagnostic).
    float unapply(float y) const {
      if (_gain == 0.0f) return NAN;
      return (_form == OFFSET_BEFORE_GAIN) ? (y / _gain - _offset)
                                           : ((y - _offset) / _gain);
    }

    // --- Lissage du brut (dépend de Smoothing) ---
    // À appeler une fois par acquisition, puis applySmoothed() pour la valeur
    // calibrée. Le lissage porte sur le BRUT (jamais sur du déjà calibré).
    void push(float raw) { _smoother.push(raw); }

    // Dernière valeur lissée (brute), méthode au choix (voir Smoothing).
    float smooth(uint8_t type, uint8_t win, float k = 3.0f, float trim = 0.15f) const {
      return _smoother.value(type, win, k, trim);
    }

    // Valeur lissée puis calibrée : apply(smooth(...)).
    float applySmoothed(uint8_t type, uint8_t win, float k = 3.0f, float trim = 0.15f) const {
      return apply(_smoother.value(type, win, k, trim));
    }

    Smoother&       smoother()       { return _smoother; }
    const Smoother& smoother() const { return _smoother; }

    // --- Résolution ---

    // 1 point : ajuste l'offset, conserve le gain.
    bool solveOnePoint(float m, float d, float& outOffset, float& outGain) const {
      outGain = _gain;
      if (_gain == 0.0f) return false;
      if (_form == OFFSET_BEFORE_GAIN) outOffset = _offset + (d - m) / _gain;
      else                             outOffset = _offset + (d - m);
      return true;
    }

    // N couples (mesuré, voulu). solveGain=false -> offset seul.
    bool solvePoints(const float* measured, const float* desired, int n,
                     bool solveGain, float& outOffset, float& outGain) const {
      if (!measured || !desired || n < 1) return false;

      if (!solveGain || n < 2) {
        return solveOnePoint(measured[0], desired[0], outOffset, outGain);
      }
      if (_gain == 0.0f) return false;   // gain nul : division impossible

      // Régression linéaire : desired = A * measured + B
      double sm = 0, sd = 0, smm = 0, smd = 0;
      for (int i = 0; i < n; i++) {
        sm  += measured[i];
        sd  += desired[i];
        smm += (double)measured[i] * measured[i];
        smd += (double)measured[i] * desired[i];
      }
      double den = (double)n * smm - sm * sm;
      if (fabs(den) < 1e-12) return false;   // points identiques -> pente indéfinie

      double A = ((double)n * smd - sm * sd) / den;
      double B = (sd - A * sm) / n;
      if (fabs(A) < 1e-12) return false;

      outGain = (float)(A * _gain);
      if (_form == OFFSET_BEFORE_GAIN) {
        outOffset = _offset + (float)(B / (A * _gain));
      } else {
        outOffset = (float)(B + A * _offset);
      }
      return true;
    }

    // Régression robuste : détecte les points aberrants via Smoothing (Smoother
    // local : push + value) puis les EXCLUT de la régression (solvePoints()).
    //   filter : filtre « conserve la valeur si elle n'est pas aberrante »
    //            (Smoother::HAMPEL ; un filtre qui renvoie toujours une moyenne,
    //            comme ZSCORE, écarterait tous les points)
    //   win    : fenêtre de détection (auto-réduite au démarrage)
    //   k      : seuil (nb de MAD pour HAMPEL)
    bool solvePointsRobust(const float* measured, const float* desired, int n,
                           bool solveGain, float& outOffset, float& outGain,
                           uint8_t filter = Smoother::HAMPEL,
                           uint8_t win = 8, float k = 3.0f) const {
      if (!measured || !desired || n < 1) return false;
      if (n > MAX_POINTS) return false;   // buffer fixe : pas d'allocation dynamique

      float mf[MAX_POINTS], df[MAX_POINTS];
      int m = 0;
      Smoother sm, sd;
      for (int i = 0; i < n; i++) {
        sm.push(measured[i]);
        sd.push(desired[i]);
        uint8_t w = (i + 1 < (int)win) ? (uint8_t)(i + 1) : win;
        float fm = sm.value(filter, w, k);
        float fd = sd.value(filter, w, k);
        // Aberrant = le filtre a remplacé la valeur (HAMPEL renvoie la valeur
        // inchangée si le point est conforme) -> on écarte le point.
        if (fm != measured[i] || fd != desired[i]) continue;
        mf[m] = measured[i];
        df[m] = desired[i];
        m++;
      }
      if (m < 1) return false;
      return solvePoints(mf, df, m, solveGain, outOffset, outGain);
    }

    // Résout puis applique.
    bool calibrate(const float* measured, const float* desired, int n, bool solveGain) {
      float o, g;
      if (!solvePoints(measured, desired, n, solveGain, o, g)) return false;
      set(o, g);
      return true;
    }

  private:
    float _offset;
    float _gain;
    Form  _form;
    Smoother _smoother;
};

#endif // CALIBRATION_H
