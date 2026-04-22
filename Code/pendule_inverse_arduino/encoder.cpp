#include "encoder.h"

const float Encoder::VELOCITY_ALPHA = 0.15f;  // filtre passe-bas (0=figé, 1=brut)

Encoder::Encoder(uint8_t pinA, uint8_t pinB, uint16_t ppr, float offset_rad)
  : _pinA(pinA), _pinB(pinB), _ppr(ppr), _offset_rad(offset_rad),
    _count(0), _prevCount(0), _lastTime_us(0), _omega(0.0f)
{
  _radsPerCount = TWO_PI / (4.0f * ppr);  // mode quadrature x4
}

void Encoder::begin() {
  pinMode(_pinA, INPUT_PULLUP);
  pinMode(_pinB, INPUT_PULLUP);
}

float Encoder::getAngleRad() const {
  // Angle brut en radians, ramené dans [-π, π]
  float angle = (float)_count * _radsPerCount + _offset_rad;
  // Normalisation dans [-π, π]
  while (angle >  PI) angle -= TWO_PI;
  while (angle < -PI) angle += TWO_PI;
  return angle;
}

float Encoder::getAngularVelocityRad() const {
  return _omega;
}

void Encoder::reset() {
  noInterrupts();
  _count      = 0;
  _prevCount  = 0;
  _omega      = 0.0f;
  _lastTime_us = micros();
  interrupts();
}

// --- ISR canal A ---
void Encoder::handleA() {
  bool a = digitalRead(_pinA);
  bool b = digitalRead(_pinB);
  // Table de vérité quadrature
  if (a == b) _count++;
  else        _count--;
  _updateVelocity();
}

// --- ISR canal B ---
void Encoder::handleB() {
  bool a = digitalRead(_pinA);
  bool b = digitalRead(_pinB);
  if (a != b) _count++;
  else        _count--;
  _updateVelocity();
}

// Méthode privée inline pour la vitesse (appelée depuis les ISR)
// Déclarée ici comme lambda-like pour rester dans le .cpp
// Note : on la déclare dans le .cpp via un petit trick
void Encoder::_updateVelocity() {
  unsigned long now = micros();
  unsigned long dt  = now - _lastTime_us;
  if (dt > 0 && dt < 100000UL) {   // ignore si dt > 100 ms (arrêt)
    int32_t dCount = _count - _prevCount;
    float raw_omega = ((float)dCount * _radsPerCount) / (dt * 1e-6f);
    _omega = VELOCITY_ALPHA * raw_omega + (1.0f - VELOCITY_ALPHA) * _omega;
  }
  _prevCount  = _count;
  _lastTime_us = now;
}
