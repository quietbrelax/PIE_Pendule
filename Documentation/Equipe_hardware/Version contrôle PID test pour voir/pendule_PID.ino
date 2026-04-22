/*
 * ============================================================
 *  PENDULE INVERSÉ — Contrôleur PID
 *  Carte : Arduino Uno
 *  Codeur : broches 2 (A) et 3 (B) — INT0 / INT1
 *  Driver pas à pas : broche 5 (PUL+), broche 6 (DIR+)
 * ============================================================
 *
 *  Commandes série (115200 bauds) :
 *    's' → active le contrôle PID
 *    'd' → désactive le contrôle (moteur à l'arrêt)
 *    'e' → arrêt d'urgence immédiat
 *    'r' → remet le compteur codeur à zéro
 *    'p' → affiche les paramètres PID actuels
 *
 * ============================================================
 */

// ─────────────────────────────────────────────
//  BROCHES
// ─────────────────────────────────────────────
#define ENC_A     2   // Codeur canal A — INT0
#define ENC_B     3   // Codeur canal B — INT1
#define STEP_PIN  5   // PUL+ driver
#define DIR_PIN   6   // DIR+ driver

// ─────────────────────────────────────────────
//  PARAMÈTRES CODEUR
// ─────────────────────────────────────────────
#define PPR             600           // impulsions/tour du codeur
#define COUNTS_PER_REV  (4 * PPR)     // quadrature x4 = 2400 counts/tour

// ─────────────────────────────────────────────
//  PARAMÈTRES MOTEUR
// ─────────────────────────────────────────────
// Pas/tour total = STEPS_PER_REV * MICROSTEPPING
// Avec 200 pas/tour et microstepping 16 → 3200 pas/tour
#define STEPS_PER_REV   200
#define MICROSTEPPING   16
#define PULLEY_DIAM_MM  113.2f        // diamètre primitif poulie (mm)

// Vitesse max du chariot (mm/s) — limite de sécurité
#define MAX_VELOCITY_MMS  400.0f

// Calcul steps/mm
const float STEPS_PER_MM = (STEPS_PER_REV * MICROSTEPPING) / (PI * PULLEY_DIAM_MM);

// ─────────────────────────────────────────────
//  PARAMÈTRES PID  — À RÉGLER
// ─────────────────────────────────────────────
//
//  Consigne : θ = 0 rad (pendule vertical vers le haut)
//  Sortie   : vitesse chariot en mm/s
//
//  GUIDE DE RÉGLAGE :
//  1. Commencez avec Kp seul (Ki=0, Kd=0), montez jusqu'à oscillations
//  2. Ajoutez Kd pour amortir (typiquement Kd = Kp * 0.1 à 0.3)
//  3. Ajoutez Ki très doucement si dérive persistante (souvent inutile)
//
float Kp = 2000.0f;   // Proportionnel — réaction à l'angle
float Ki =    0.0f;   // Intégral     — corrige les dérives lentes
float Kd =  200.0f;   // Dérivé       — amortit les oscillations

// Anti-windup : limite l'accumulation de l'intégrale
#define INTEGRAL_LIMIT  50.0f   // mm/s max de contribution intégrale

// Filtre passe-bas sur le terme dérivé (évite l'amplification du bruit)
// alpha = 1.0 → pas de filtre ; alpha = 0.1 → très filtré
#define DERIVATIVE_ALPHA  0.2f

// Lissage de la commande finale (évite les à-coups moteur)
#define COMMAND_ALPHA     0.4f

// Zone morte angulaire : en dessous de ce seuil, commande = 0
// Utile pour compenser les frottements secs au pivot
#define DEADBAND_RAD      0.01f   // ≈ 0.6°

// Zone d'activation : au-delà de cet angle, le contrôle se coupe
#define MAX_ANGLE_RAD     0.5f    // ≈ 28° — arrêt si pendule trop incliné

// ─────────────────────────────────────────────
//  PÉRIODE D'ÉCHANTILLONNAGE
// ─────────────────────────────────────────────
#define CONTROL_PERIOD_US  5000UL   // 5 ms → 200 Hz

// ─────────────────────────────────────────────
//  VARIABLES GLOBALES
// ─────────────────────────────────────────────

// Codeur
volatile int32_t encCount = 0;

// États PID
float   integralTerm   = 0.0f;
float   prevError      = 0.0f;
float   derivFiltered  = 0.0f;
float   cmdFiltered    = 0.0f;
unsigned long lastControlTime = 0;

// Générateur de pas moteur
volatile uint32_t stepInterval_us = 0;   // intervalle entre deux pas (µs), 0 = arrêt
volatile bool     stepDir         = true;
volatile uint32_t lastStepTime    = 0;

// État du système
bool controlActive    = false;
bool emergencyStop    = false;

// ─────────────────────────────────────────────
//  ISR CODEUR
// ─────────────────────────────────────────────
void isrA() {
  if (digitalRead(ENC_A) == digitalRead(ENC_B)) encCount++;
  else                                           encCount--;
}

void isrB() {
  if (digitalRead(ENC_A) == digitalRead(ENC_B)) encCount--;
  else                                           encCount++;
}

// ─────────────────────────────────────────────
//  UTILITAIRES
// ─────────────────────────────────────────────

// Lecture thread-safe du compteur codeur
int32_t readEncoder() {
  noInterrupts();
  int32_t c = encCount;
  interrupts();
  return c;
}

// Convertit un compteur codeur en radians, normalisé dans [-π, π]
float countsToRad(int32_t counts) {
  float angle = (float)counts / COUNTS_PER_REV * TWO_PI;
  while (angle >  PI) angle -= TWO_PI;
  while (angle < -PI) angle += TWO_PI;
  return angle;
}

// Envoie la commande vitesse au moteur (mm/s)
// Génère les impulsions STEP dans loop() via stepInterval_us
void setMotorVelocity(float vel_mms) {
  if (vel_mms == 0.0f || emergencyStop) {
    stepInterval_us = 0;
    return;
  }

  // Direction
  stepDir = (vel_mms > 0.0f);
  digitalWrite(DIR_PIN, stepDir ? HIGH : LOW);

  // Intervalle entre deux pas
  float speed_steps_s = abs(vel_mms) * STEPS_PER_MM;
  if (speed_steps_s < 1.0f) { stepInterval_us = 0; return; }

  stepInterval_us = (uint32_t)(1000000.0f / speed_steps_s);
}

// Arrêt immédiat
void stopMotor() {
  stepInterval_us = 0;
}

// ─────────────────────────────────────────────
//  SETUP
// ─────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  // Codeur
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A), isrA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), isrB, CHANGE);

  // Moteur
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN,  OUTPUT);
  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN,  LOW);

  Serial.println(F("=== Pendule Inverse - PID ==="));
  Serial.print  (F("steps/mm = "));
  Serial.println(STEPS_PER_MM, 3);
  Serial.println(F("Kp=")); Serial.print(Kp);
  Serial.print  (F("  Ki=")); Serial.print(Ki);
  Serial.print  (F("  Kd=")); Serial.println(Kd);
  Serial.println(F("-------------------------------"));
  Serial.println(F("'s' = activer   'd' = desactiver   'e' = urgence   'r' = reset codeur   'p' = params"));
}

// ─────────────────────────────────────────────
//  BOUCLE PID
// ─────────────────────────────────────────────
void controlLoop() {
  unsigned long now = micros();
  if ((now - lastControlTime) < CONTROL_PERIOD_US) return;
  float dt = (now - lastControlTime) * 1e-6f;
  lastControlTime = now;

  // ── Lecture angle ──────────────────────────
  float theta = countsToRad(readEncoder());

  // ── Vérification zone de contrôle ─────────
  if (abs(theta) > MAX_ANGLE_RAD) {
    stopMotor();
    controlActive = false;
    Serial.print(F("[FAULT] Angle trop grand : "));
    Serial.print(theta * RAD_TO_DEG, 1);
    Serial.println(F(" deg. Envoyez 's' pour relancer."));
    return;
  }

  // ── Erreur (consigne = 0 rad) ──────────────
  float error = -theta;   // on veut theta = 0

  // Zone morte : ignore les très petites erreurs (frottements secs)
  if (abs(error) < DEADBAND_RAD) error = 0.0f;

  // ── Terme proportionnel ────────────────────
  float pTerm = Kp * error;

  // ── Terme intégral avec anti-windup ────────
  integralTerm += Ki * error * dt;
  integralTerm  = constrain(integralTerm, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

  // ── Terme dérivé filtré ────────────────────
  float rawDeriv  = (error - prevError) / dt;
  derivFiltered   = DERIVATIVE_ALPHA * rawDeriv
                  + (1.0f - DERIVATIVE_ALPHA) * derivFiltered;
  float dTerm     = Kd * derivFiltered;
  prevError       = error;

  // ── Commande brute ─────────────────────────
  float u = pTerm + integralTerm + dTerm;
  u = constrain(u, -MAX_VELOCITY_MMS, MAX_VELOCITY_MMS);

  // ── Lissage commande (anti à-coups) ────────
  cmdFiltered = COMMAND_ALPHA * u + (1.0f - COMMAND_ALPHA) * cmdFiltered;

  // ── Envoi au moteur ────────────────────────
  setMotorVelocity(cmdFiltered);

  // ── Télémétrie (décommentez pour debug) ────
  // Serial.print(theta * RAD_TO_DEG, 2);
  // Serial.print('\t'); Serial.print(pTerm, 1);
  // Serial.print('\t'); Serial.print(dTerm, 1);
  // Serial.print('\t'); Serial.println(cmdFiltered, 1);
}

// ─────────────────────────────────────────────
//  GÉNÉRATEUR DE PAS MOTEUR (appelé dans loop)
// ─────────────────────────────────────────────
void stepMotor() {
  if (stepInterval_us == 0) return;
  uint32_t now = micros();
  if ((now - lastStepTime) >= stepInterval_us) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(2);           // pulse minimum 1 µs pour DM542T
    digitalWrite(STEP_PIN, LOW);
    lastStepTime = now;
  }
}

// ─────────────────────────────────────────────
//  GESTION DES COMMANDES SÉRIE
// ─────────────────────────────────────────────
void handleSerial() {
  if (!Serial.available()) return;
  char c = Serial.read();

  switch (c) {
    case 's': case 'S':
      if (!emergencyStop) {
        // Réinitialise l'état PID
        integralTerm  = 0.0f;
        prevError     = 0.0f;
        derivFiltered = 0.0f;
        cmdFiltered   = 0.0f;
        lastControlTime = micros();
        controlActive = true;
        Serial.println(F("[ON] Controle PID actif."));
      } else {
        Serial.println(F("[!] Urgence active. Appuyez 'r' puis 's'."));
      }
      break;

    case 'd': case 'D':
      controlActive = false;
      stopMotor();
      Serial.println(F("[OFF] Controle desactive."));
      break;

    case 'e': case 'E':
      emergencyStop = true;
      controlActive = false;
      stopMotor();
      Serial.println(F("[URGENCE] Arret immediat ! Envoyez 'r' pour reinitialiser."));
      break;

    case 'r': case 'R':
      emergencyStop = false;
      noInterrupts();
      encCount = 0;
      interrupts();
      integralTerm  = 0.0f;
      prevError     = 0.0f;
      derivFiltered = 0.0f;
      cmdFiltered   = 0.0f;
      stopMotor();
      Serial.println(F("[RESET] Codeur et PID reinitialises."));
      break;

    case 'p': case 'P':
      Serial.print(F("Kp=")); Serial.print(Kp);
      Serial.print(F("  Ki=")); Serial.print(Ki);
      Serial.print(F("  Kd=")); Serial.print(Kd);
      Serial.print(F("  angle="));
      Serial.print(countsToRad(readEncoder()) * RAD_TO_DEG, 2);
      Serial.println(F(" deg"));
      break;
  }
}

// ─────────────────────────────────────────────
//  LOOP
// ─────────────────────────────────────────────
void loop() {
  // Génération des pas moteur — en priorité, ne pas bloquer
  stepMotor();

  // Commandes clavier PC via moniteur série
  handleSerial();

  // Boucle de contrôle PID
  if (controlActive && !emergencyStop) {
    controlLoop();
  }
}
