#include "stepper_velocity.h"

StepperVelocity::StepperVelocity(uint8_t stepPin, uint8_t dirPin,
                                  float stepsPerMm,
                                  float maxSpeed_mms,
                                  float maxAccel_mms2)
  : _stepper(AccelStepper::DRIVER, stepPin, dirPin),
    _stepsPerMm(stepsPerMm),
    _currentVel_mms(0.0f),
    _limitsEnabled(false),
    _minPos_mm(-1e9f), _maxPos_mm(1e9f)
{
  _maxSpeed_steps = maxSpeed_mms * stepsPerMm;
  _stepper.setMaxSpeed(_maxSpeed_steps);
  // En mode vitesse, setAcceleration n'est pas utilisé par runSpeed(),
  // mais on le garde pour d'éventuels mouvements en position.
  _stepper.setAcceleration(maxAccel_mms2 * stepsPerMm);
}

void StepperVelocity::begin() {
  _stepper.setCurrentPosition(0);
  _stepper.setSpeed(0);
}

void StepperVelocity::setVelocity(float vel_mms) {
  // Clamp à la vitesse max
  vel_mms = constrain(vel_mms, -(_maxSpeed_steps / _stepsPerMm),
                                 (_maxSpeed_steps / _stepsPerMm));

  // Fins de course logiciels
  if (_limitsEnabled) {
    float posMm = getPositionMm();
    if (posMm <= _minPos_mm && vel_mms < 0.0f) vel_mms = 0.0f;
    if (posMm >= _maxPos_mm && vel_mms > 0.0f) vel_mms = 0.0f;
  }

  _currentVel_mms = vel_mms;
  _stepper.setSpeed(mmToSteps(vel_mms));
}

void StepperVelocity::update() {
  // runSpeed() avance d'un pas si le timing l'exige — ne bloque pas
  _stepper.runSpeed();
}

float StepperVelocity::getPositionMm() const {
  return stepsToMm((float)_stepper.currentPosition());
}

void StepperVelocity::resetPosition() {
  _stepper.setCurrentPosition(0);
}

void StepperVelocity::stop() {
  _currentVel_mms = 0.0f;
  _stepper.setSpeed(0.0f);
}

void StepperVelocity::setSoftLimits(float minMm, float maxMm) {
  _minPos_mm     = minMm;
  _maxPos_mm     = maxMm;
  _limitsEnabled = true;
}
