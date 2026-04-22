/**
 * ============================================================
 *  PENDULE INVERSÉ — Contrôle LQR sur chariot linéaire
 * ============================================================
 *
 * Matériel supposé :
 *   - Arduino Mega (recommandé : 6 broches d'interruption)
 *     ou ESP32 (toutes les broches supportent les interruptions)
 *   - Codeur incrémental quadrature sur broches 2 & 3 (INT0/INT1 Mega)
 *   - Driver pas à pas (A4988/DRV8825/TMC2209) sur broches STEP/DIR
 *   - Fin de course optionnel sur broche LIMIT_PIN
 *
 * Bibliothèques requises :
 *   - AccelStepper  → Gestionnaire de bibliothèques Arduino
 *
 * Contrôleur :
 *   Retour d'état LQR linéarisé autour de l'équilibre instable (θ=0)
 *   u = -K * [θ, θ̇, x, ẋ]
 *   où u est la vitesse du chariot en mm/s.
 *
 *   Les gains K sont à régler selon votre système physique.
 *   Un point de départ raisonnable est fourni — ajustez par simulation
 *   ou par essais (voir section RÉGLAGE DES GAINS).
 *
 * Repère angulaire :
 *   θ = 0   → pendule vertical vers le HAUT (équilibre instable)
 *   θ > 0   → pendule penché à droite
 *   Le codeur doit être câblé pour que ce sens soit respecté.
 * ============================================================
 

#include "encoder.h"
#include "stepper_velocity.h"

// ─────────────────────────────────────────────
//  BROCHES
// ─────────────────────────────────────────────
constexpr uint8_t ENC_A       = 2;   // Interruption INT0
constexpr uint8_t ENC_B       = 3;   // Interruption INT1
constexpr uint8_t STEP_PIN    = 5;
constexpr uint8_t DIR_PIN     = 6; 
constexpr uint8_t ENABLE_PIN  = 7;   // LOW = driver activé (A4988/DRV8825)
constexpr uint8_t LIMIT_PIN   = 8;   // fin de course (INPUT_PULLUP, NC)

// ─────────────────────────────────────────────
//  PARAMÈTRES MÉCANIQUES  — À ADAPTER
// ─────────────────────────────────────────────
// Codeur
constexpr uint16_t ENC_PPR        = 600;    // impulsions/tour (avant x4)
constexpr float    ENC_OFFSET_RAD = 0.0f;    // offset si le zéro n'est pas au top

float u_filtered = 0.0f;          // commande lissée
constexpr float U_ALPHA = 0.25;   // 0.0 = immobile, 1.0 = pas de lissage
                                   // commencez à 0.3, montez si trop mou

// Moteur / transmission
constexpr float STEPS_PER_REV  = 200.0f;    // pas/tour moteur (1.8°)
constexpr float MICROSTEPPING  = 16.0f;     // microstep configuré sur le driver
constexpr float PULLEY_DIAM_MM = 113.2f;     // diamètre primitif poulie crantée (mm)
// steps/mm = (steps/rev * microstepping) / (π * Ø)
const float STEPS_PER_MM = (STEPS_PER_REV * MICROSTEPPING)
                           / (PI * PULLEY_DIAM_MM);   // ≈ 50.9 steps/mm pour GT2 Ø20

// Limites du chariot (mm depuis la position zéro, trouvée à l'init)
constexpr float RAIL_HALF_LENGTH_MM = 300.0f;

// ─────────────────────────────────────────────
//  PARAMÈTRES DU CONTRÔLEUR LQR  — À RÉGLER
// ─────────────────────────────────────────────
//
//  RÉGLAGE DES GAINS :
//  Le modèle linéarisé du pendule sur chariot est :
//    θ̈ = (g/l)·θ  +  (1/l)·ü_chariot   (approximation sin θ ≈ θ)
//  avec l = longueur du pendule (m), g = 9.81 m/s²
//
//  Pour régler par simulation (Python/Matlab) :
//    A = [[0,1,0,0],[g/l,0,0,0],[0,0,0,1],[0,0,0,0]]
//    B = [[0],[-1/l],[0],[1]]
//    Q = diag([q_theta, q_dtheta, q_x, q_dx])
//    R = [r_u]
//  puis K = lqr(A, B, Q, R)
//
//  Point de départ empirique (pendule ~30 cm, rail ~300 mm) :
//    K = [Kθ, Kθ̇, Kx, Kẋ]  →  u_vitesse = -K * état
//
constexpr float PENDULUM_LENGTH_M = 0.0152f;   // longueur pendule (m) — à mesurer !

// Gains LQR (vitesse chariot en mm/s par unité d'état)
// θ en rad, θ̇ en rad/s, x en mm, ẋ en mm/s
constexpr float K_THETA  =  -3162.0f;   // [mm/s / rad]
constexpr float K_DTHETA =  -6377.0f;   // [mm/s / (rad/s)]
constexpr float K_X      =  -0.0867f;   // [mm/s / mm]     (maintien de position)
constexpr float K_DX     =  -0.05406f;   // [mm/s / (mm/s)]

// Saturation de la commande
constexpr float MAX_VELOCITY_MMS  = 7000.0f;  // mm/s

// Zone d'activation du contrôle (en dehors = arrêt moteur)
// Si le pendule est trop loin de la verticale, on abandonne
constexpr float CONTROL_ANGLE_RAD = 1.50f;   // ≈ 20°

// ─────────────────────────────────────────────
//  PÉRIODE D'ÉCHANTILLONNAGE
// ─────────────────────────────────────────────
constexpr unsigned long CONTROL_PERIOD_US = 5000UL;   // 5 ms → 200 Hz

// ─────────────────────────────────────────────
//  OBJETS GLOBAUX
// ─────────────────────────────────────────────
Encoder        enc(ENC_A, ENC_B, ENC_PPR, ENC_OFFSET_RAD);
StepperVelocity motor(STEP_PIN, DIR_PIN, STEPS_PER_MM, MAX_VELOCITY_MMS);

// ISR globales — obligatoire pour les méthodes d'instance
void isrA() { enc.handleA(); }
void isrB() { enc.handleB(); }

// ─────────────────────────────────────────────
//  VARIABLES D'ÉTAT
// ─────────────────────────────────────────────
enum State { INIT, HOMING, WAITING, CONTROLLING, FAULT };
State systemState = INIT;

float x_ref_mm = 0.0f;           // consigne de position chariot (centre rail)

// Filtrage simple de la vitesse du chariot (dérivée de position)
float x_prev_mm  = 0.0f;
float dx_filtered = 0.0f;
constexpr float DX_ALPHA = 0.2f;

unsigned long lastControlTime_us = 0;

// ─────────────────────────────────────────────
//  SETUP
// ─────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  enc.begin();
  attachInterrupt(digitalPinToInterrupt(ENC_A), isrA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), isrB, CHANGE);

  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, HIGH);  // driver désactivé au départ

  motor.begin();
  motor.setSoftLimits(-RAIL_HALF_LENGTH_MM, RAIL_HALF_LENGTH_MM);

  pinMode(LIMIT_PIN, INPUT_PULLUP);

  Serial.println(F("=== Pendule Inverse - LQR ==="));
  Serial.print(F("stalcule = "));
  Serial.println(STEPS_PER_MM, 3);
  Serial.println(F("Envoyez 'h' pour homing."));

  systemState = HOMING;
}

// ─────────────────────────────────────────────
//  HOMING : trouve le centre du rail
// ─────────────────────────────────────────────
void doHoming() {
  Serial.println(F("[HOMING] Centrez le chariot manuellement sur le rail."));
  Serial.println(F("Puis envoyez 'c' pour confirmer la position zero."));

  // Attend la confirmation
  while (true) {
    if (Serial.available()) {
      char c = Serial.read();
      if (c == 'c' || c == 'C') break;
    }
  }

  motor.resetPosition();
  x_ref_mm = 0.0f;
  Serial.println(F("[HOMING] Zero defini. Envoyez 's' pour demarrer le controle."));
  systemState = WAITING;
}

// ─────────────────────────────────────────────
//  BOUCLE DE CONTRÔLE LQR
// ─────────────────────────────────────────────
void controlLoop() {
  unsigned long now_us = micros();
  if ((now_us - lastControlTime_us) < CONTROL_PERIOD_US) return;
  lastControlTime_us = now_us;

  // ── Lecture des états ──────────────────────
  float theta  = enc.getAngleRad();          // rad
  float dtheta = enc.getAngularVelocityRad(); // rad/s
  float x      = motor.getPositionMm();       // mm
  float dx_raw = (x - x_prev_mm) / (CONTROL_PERIOD_US * 1e-6f);
  dx_filtered  = DX_ALPHA * dx_raw + (1.0f - DX_ALPHA) * dx_filtered;
  x_prev_mm    = x;

  // ── Vérification zone de contrôle ─────────
  if (abs(theta) > CONTROL_ANGLE_RAD) {
    motor.stop();
    digitalWrite(ENABLE_PIN, HIGH);
    systemState = FAULT;
    Serial.print(F("[FAULT] Angle trop grand : "));
    Serial.print(theta * RAD_TO_DEG, 1);
    Serial.println(F("°. Relancer avec 's'."));
    return;
  }

  // ── Loi LQR : u = -K * [θ, θ̇, x-x_ref, ẋ] ──
  float u = -(K_THETA  * theta
            + K_DTHETA * dtheta
            + K_X      * (x - x_ref_mm)
            + K_DX     * dx_filtered);

  // Saturation
  u = constrain(u, -MAX_VELOCITY_MMS, MAX_VELOCITY_MMS);
  u_filtered = U_ALPHA * u + (1.0f - U_ALPHA) * u_filtered;
  motor.setVelocity(u_filtered);
  motor.update();

  // ── Télémétrie série (décommenter si besoin) ──
  //Format : theta(deg), dtheta(rad/s), x(mm), u(mm/s)
  Serial.print(theta * RAD_TO_DEG, 3);
  Serial.print('\t'); Serial.print(dtheta, 3);
  Serial.print('\t'); Serial.print(x, 2);
  Serial.print('\t'); Serial.println(u, 1);
}

// ─────────────────────────────────────────────
//  LOOP PRINCIPALE
// ─────────────────────────────────────────────
void loop() {
  // Toujours faire tourner les steps moteur en priorité
  motor.update();

  // ── Commandes série ────────────────────────
  if (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case 'h':
      case 'H':
        systemState = HOMING;
        break;

      case 's':
      case 'S':
        if (systemState == WAITING || systemState == FAULT) {
          enc.reset();
          motor.resetPosition();
          x_prev_mm    = 0.0f;
          dx_filtered  = 0.0f;
          lastControlTime_us = micros();
          digitalWrite(ENABLE_PIN, LOW);  // active le driver
          systemState = CONTROLLING;
          Serial.println(F("[CTRL] Contrôle LQR actif."));
        }
        break;

      case 'e':
      case 'E':
        motor.stop();
        digitalWrite(ENABLE_PIN, HIGH);
        systemState = WAITING;
        Serial.println(F("[STOP] Arrêt d'urgence."));
        break;

      case 'p':
      case 'P':
        // Affichage des paramètres
        Serial.print(F("theta = ")); Serial.print(enc.getAngleRad() * RAD_TO_DEG, 2);
        Serial.print(F("°  dtheta = ")); Serial.print(enc.getAngularVelocityRad(), 3);
        Serial.print(F(" rad/s  x = ")); Serial.print(motor.getPositionMm(), 1);
        Serial.println(F(" mm"));
        break;
    }
  }

  // ── Machine d'état ────────────────────────
  switch (systemState) {
    case HOMING:
      doHoming();
      break;

    case CONTROLLING:
      controlLoop();
      break;

    case WAITING:
    case FAULT:
    case INIT:
    default:
      break;
  }
}
**/

// ── Broches codeur ──────────────────────
#define ENC_A 2   // INT0
#define ENC_B 3   // INT1

// ── Résolution ──────────────────────────
#define PPR 600         // impulsions/tour de votre codeur
#define COUNTS_PER_REV (4 * PPR)   // mode quadrature x4

// ── Variables volatiles (modifiées dans les ISR) ──
volatile int32_t count = 0;

// ── ISR ─────────────────────────────────
void isrA() {
  if (digitalRead(ENC_A) == digitalRead(ENC_B)) count++;
  else count--;
}

void isrB() {
  if (digitalRead(ENC_A) == digitalRead(ENC_B)) count--;
  else count++;
}

void setup() {
  Serial.begin(115200);
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A), isrA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), isrB, CHANGE);
}

void loop() {
  static int32_t prevCount = 0;
  static unsigned long prevTime = 0;

  unsigned long now = millis();
  if (now - prevTime >= 50) {   // affichage toutes les 50 ms
    int32_t c = count;          // lecture atomique
    float angle_deg = (float)c / COUNTS_PER_REV * 360.0;
    float angle_rad = (float)c / COUNTS_PER_REV * TWO_PI;

    // Vitesse en rad/s sur la dernière fenêtre de 50 ms
    float dt = (now - prevTime) / 1000.0;
    float omega = ((float)(c - prevCount) / COUNTS_PER_REV * TWO_PI) / dt;

    Serial.print("counts="); Serial.print(c);
    Serial.print("  deg=");  Serial.print(angle_deg, 2);
    Serial.print("  rad=");  Serial.print(angle_rad, 4);
    Serial.print("  omega="); Serial.print(omega, 3);
    Serial.println(" rad/s");

    prevCount = c;
    prevTime  = now;
  }
}
