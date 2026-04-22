#pragma once
#include <Arduino.h>
#include <AccelStepper.h>

/**
 * Contrôle en vitesse d'un moteur pas à pas via AccelStepper.
 * On impose directement une consigne de vitesse (mm/s ou steps/s)
 * sans gérer manuellement les pas de temps.
 *
 * AccelStepper est utilisé en mode vitesse constante (runSpeed),
 * avec une accélération max pour les transitions.
 *
 * Bibliothèque requise : AccelStepper (Mike McCaulay)
 * → Gestionnaire de bibliothèques Arduino : "AccelStepper"
 */
class StepperVelocity {
public:
  /**
   * @param stepPin     Broche STEP du driver (A4988, DRV8825, TMC2209…)
   * @param dirPin      Broche DIR du driver
   * @param stepsPerMm  Pas moteur par mm de déplacement chariot
   *                    = (steps/rev * microstepping) / (π * pulley_diameter_mm)
   *                    Ex: 200 * 16 / (π * 20) ≈ 50.9 steps/mm pour poulie GT2 Ø20mm
   * @param maxSpeed_mms   Vitesse max en mm/s
   * @param maxAccel_mms2  Accélération max en mm/s² (pour les changements de vitesse)
   */
  StepperVelocity(uint8_t stepPin, uint8_t dirPin,
                  float stepsPerMm,
                  float maxSpeed_mms  = 500.0f,
                  float maxAccel_mms2 = 2000.0f);

  void begin();

  /**
   * Définit la consigne de vitesse en mm/s.
   * Signe : positif = droite, négatif = gauche (selon câblage DIR).
   * À appeler à chaque itération du contrôleur.
   */
  void setVelocity(float vel_mms);

  /**
   * Met à jour le moteur — DOIT être appelé le plus souvent possible
   * dans loop() ou une ISR timer. Ne bloque pas.
   */
  void update();

  // Vitesse actuelle commandée (mm/s)
  float getCurrentVelocity() const { return _currentVel_mms; }

  // Position absolue en mm depuis le reset (intégrée depuis les steps)
  float getPositionMm() const;

  // Remet la position à zéro (ne bouge pas le moteur)
  void resetPosition();

  // Arrêt d'urgence immédiat
  void stop();

  // Limite de position logicielle (fin de course soft) en mm
  void setSoftLimits(float minMm, float maxMm);

private:
  AccelStepper _stepper;
  float        _stepsPerMm;
  float        _maxSpeed_steps;
  float        _currentVel_mms;

  // Fins de course logiciels
  float _minPos_mm, _maxPos_mm;
  bool  _limitsEnabled;

  float mmToSteps(float mm) const { return mm * _stepsPerMm; }
  float stepsToMm(float s)  const { return s  / _stepsPerMm; }
};
