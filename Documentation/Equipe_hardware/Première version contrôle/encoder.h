#pragma once
#include <Arduino.h>

/**
 * Codeur incrémental en quadrature (A/B)
 * Utilise deux interruptions matérielles pour un comptage précis.
 * La position est convertie en radians selon la résolution du codeur.
 */
class Encoder {
public:
  /**
   * @param pinA       Broche canal A (doit supporter les interruptions)
   * @param pinB       Broche canal B (doit supporter les interruptions)
   * @param ppr        Pulses Per Revolution (impulsions par tour du codeur)
   *                   → nombre de fronts comptés = 4 * ppr (mode quadrature x4)
   * @param offset_rad Offset angulaire en radians (pour caler le zéro)
   */
  Encoder(uint8_t pinA, uint8_t pinB, uint16_t ppr, float offset_rad = 0.0f);

  void begin();

  // Position en radians (0 = pendule vers le haut, positif = sens horaire)
  float getAngleRad() const;

  // Vitesse angulaire en rad/s (dérivée filtrée)
  float getAngularVelocityRad() const;

  // Remet le compteur à zéro (ou à l'offset)
  void reset();

  // Appelé par les ISR — ne pas appeler manuellement
  void handleA();
  void handleB();

private:
  uint8_t  _pinA, _pinB;
  uint16_t _ppr;
  float    _offset_rad;
  float    _radsPerCount;   // 2π / (4 * ppr)

  volatile int32_t _count;
  volatile int32_t _prevCount;

  // Pour le calcul de vitesse
  volatile unsigned long _lastTime_us;
  volatile float         _omega;        // rad/s filtré

  static const float VELOCITY_ALPHA;   // coefficient filtre passe-bas

  void _updateVelocity();    // appelé depuis handleA/handleB
};
