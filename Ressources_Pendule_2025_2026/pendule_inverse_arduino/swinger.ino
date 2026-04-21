/*
// ============================================================
//  Swing-Up pendule inverse — Méthode par Énergie (Rail 1m)
//  Style fluide (comme sur la vidéo)
// ============================================================

#include "encoder.h"
#include "stepper_velocity.h"

// ── Broches ──────────────────────────────────────────────────
#define ENC_PIN_A   2
#define ENC_PIN_B   3
#define STEP_PIN    4
#define DIR_PIN     5
#define ENABLE_PIN  7   // LOW = driver activé

// ── Paramètres hardware ──────────────────────────────────────
const float PPR          = 600.0f;
const float STEPS_PER_MM = 50.9f;
const float MAX_SPEED    = 800.0f;   // mm/s (Vitesse max bien fluide)
const float MAX_ACCEL    = 4000.0f;  // mm/s² (Bonne accélération sans être trop violente)
const float CART_LIMIT   = 400.0f;   // ±450 mm depuis le centre (Rail de 1000mm !)

// ── Paramètres physiques du pendule ──────────────────────────
// ⚠️ IMPORTANT : Mets tes vraies valeurs ici si tu les connais pour optimiser
const float PENDULUM_MASS = 0.1f;   // kg  
const float PENDULUM_LEN  = 0.15f;  // m   (Distance de l'axe au centre de gravité)
const float G             = 9.81f;

// ── Paramètres swing-up ──────────────────────────────────────
const float E_TARGET = 2.0f * PENDULUM_MASS * G * PENDULUM_LEN; // Énergie cible (en haut)
const float KICK_SPEED = 600.0f;        // Vitesse de "pompage" 
const float UPRIGHT_THRESHOLD = 0.35f;  // rad (environ ±20° du sommet pour s'arrêter)

// ── Objets ───────────────────────────────────────────────────
Encoder         pendulumEnc(ENC_PIN_A, ENC_PIN_B, PPR, 0.0f);
StepperVelocity cart(STEP_PIN, DIR_PIN, STEPS_PER_MM, MAX_SPEED, MAX_ACCEL);

void isrA() { pendulumEnc.handleA(); }
void isrB() { pendulumEnc.handleB(); }

enum State { WAITING, RUNNING, DONE, FAULT };
State state = WAITING;

void setup() {
  Serial.begin(115200);

  pendulumEnc.begin();
  attachInterrupt(digitalPinToInterrupt(ENC_PIN_A), isrA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_PIN_B), isrB, CHANGE);

  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, HIGH);  

  cart.begin();
  cart.setSoftLimits(-CART_LIMIT, CART_LIMIT);

  printHelp();
}

void loop() {
  cart.update();  

  handleSerial();

  if (state == RUNNING) {
    runSwingUp();
  }
}

// ── Swing-up principal ────────────────────────────────────────
void runSwingUp() {
  float theta   = pendulumEnc.getAngleRad();
  float omega   = pendulumEnc.getAngularVelocityRad();
  float cartPos = cart.getPositionMm();

  // 1. Détection arrivée en haut
  float angleFromTop = abs(abs(theta) - PI);
  if (angleFromTop < UPRIGHT_THRESHOLD) {
    cart.stop();
    digitalWrite(ENABLE_PIN, HIGH); // Coupe les moteurs. C'est ici que tu mettras ton code de stabilisation LQR.
    state = DONE;
    Serial.println(F("\n>>> Pendule en haut ! Swing-up termine. <<<"));
    Serial.println(F("Envoyez 'r' pour recommencer."));
    return;
  }

  // 2. Calcul de l'énergie mécanique actuelle
  float E = 0.5f * PENDULUM_MASS * PENDULUM_LEN * PENDULUM_LEN * omega * omega
          + PENDULUM_MASS * G * PENDULUM_LEN * (1.0f - cos(theta));

  // 3. Logique d'injection d'énergie (Pompage)
// 3. Logique d'injection d'énergie (Pompage fluide)
  float direction = omega * cos(theta);
  float vel_cmd = 0.0f;

  // Coup d'amorçage : On pousse fort UNIQUEMENT si le pendule est au centre
  // Et on utilise cartPos pour s'assurer que le chariot bouge vraiment avant de couper l'amorçage
  if (abs(omega) < 0.3f && abs(theta) < 0.2f) {
     vel_cmd = KICK_SPEED;
  }
  else if (E < E_TARGET) {
    // Il manque de l'énergie : on accélère dans le sens inverse du mouvement du pendule
    vel_cmd = (direction > 0.0f) ? -KICK_SPEED : KICK_SPEED;
  } else {
    // Trop d'énergie : on freine doucement
    vel_cmd = (direction > 0.0f) ? KICK_SPEED * 0.5f : -KICK_SPEED * 0.5f;
  }

  // 4. Protection des bords (Rail de 1m)
  // On freine doucement si on dépasse les ±400mm pour recentrer l'action
  if (cartPos > 300.0f && vel_cmd > 0.0f) {
    vel_cmd = -KICK_SPEED * 0.3f; 
  } else if (cartPos < -400.0f && vel_cmd < 0.0f) {
    vel_cmd = KICK_SPEED * 0.3f;
  }

  cart.setVelocity(vel_cmd);

  // Debug série (toutes les 100 ms)
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 100) {
    lastPrint = millis();
    Serial.print(F("theta=")); Serial.print(theta * RAD_TO_DEG, 1);
    Serial.print(F("deg  E=")); Serial.print(E, 4);
    Serial.print(F("  cart=")); Serial.print(cartPos, 1);
    Serial.println(F("mm"));
  }
}

// ── Gestion commandes série ───────────────────────────────────
void handleSerial() {
  if (!Serial.available()) return;
  char c = Serial.read();

  switch (c) {
    case 's': case 'S':
      if (state == WAITING || state == FAULT) {
        digitalWrite(ENABLE_PIN, LOW);
        pendulumEnc.reset();
        cart.resetPosition(); // LE CHARIOT DOIT ETRE AU CENTRE DU RAIL
        state = RUNNING;
        Serial.println(F("[GO] Swing-up fluide demarre !"));
      }
      break;

    case 'e': case 'E':
      cart.stop();
      digitalWrite(ENABLE_PIN, HIGH);
      state = WAITING;
      Serial.println(F("[STOP] Arret d'urgence."));
      break;

    case 'r': case 'R':
      cart.stop();
      digitalWrite(ENABLE_PIN, HIGH);
      pendulumEnc.reset();
      cart.resetPosition();
      state = WAITING;
      Serial.println(F("[RESET] Pret. N'oubliez pas de CENTRER le chariot physiquement avant 's'."));
      break;
  }
}

void printHelp() {
  Serial.println(F("================================"));
  Serial.println(F("  Swing-Up Energie (Rail 1m)"));
  Serial.println(F("================================"));
  Serial.println(F("  s -> Demarrer"));
  Serial.println(F("  e -> Arret d'urgence"));
  Serial.println(F("  r -> Reset (centrer chariot avant)"));
  Serial.println(F("================================"));
}
*/

// ============================================================
//  Swing-Up pendule inverse — Méthode Simple (Coup à 0) 
//  AVEC ANTI-REBOND (Cooldown)
// ============================================================

#include "encoder.h"
#include "stepper_velocity.h"

// ── Broches ──────────────────────────────────────────────────
#define ENC_PIN_A   2
#define ENC_PIN_B   3
#define STEP_PIN    4
#define DIR_PIN     5
#define ENABLE_PIN  7   

// ── Paramètres hardware ──────────────────────────────────────
const float PPR          = 600.0f;
const float STEPS_PER_MM = 50.9f;    
const float MAX_SPEED    = 800.0f;   
const float MAX_ACCEL    = 6000.0f;  

// ── Paramètres du coup ("Kick") ──────────────────────────────
const float KICK_SPEED = 800.0f;           
const unsigned long KICK_DURATION = 200;   
const float UPRIGHT_THRESHOLD = 0.35f;

// ── Objets ───────────────────────────────────────────────────
Encoder         pendulumEnc(ENC_PIN_A, ENC_PIN_B, PPR, 0.0f);
StepperVelocity cart(STEP_PIN, DIR_PIN, STEPS_PER_MM, MAX_SPEED, MAX_ACCEL);

void isrA() { pendulumEnc.handleA(); }
void isrB() { pendulumEnc.handleB(); }

enum State { WAITING, RUNNING, DONE };
State state = WAITING;

// Variables pour retenir le passage à zéro et le cooldown
bool wasPositive = true;
bool isKicking = false;
unsigned long kickTimer = 0;
unsigned long lastZeroTime = 0; // Chrono pour l'anti-bruit

void setup() {
  Serial.begin(115200);
  pendulumEnc.begin();
  attachInterrupt(digitalPinToInterrupt(ENC_PIN_A), isrA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_PIN_B), isrB, CHANGE);
  
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, HIGH);  

  cart.begin();
  cart.setSoftLimits(-450.0f, 450.0f); 

  Serial.println(F("Code simplifie pret."));
  Serial.println(F("Placez au centre et tapez 's' pour lancer."));
}

void loop() {
  cart.update();  

  // Commandes série
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 's') {
      digitalWrite(ENABLE_PIN, LOW);
      pendulumEnc.reset();
      cart.resetPosition();
      wasPositive = true;
      isKicking = false;
      state = RUNNING;
      
      // Coup de départ manuel pour lancer l'oscillation
      cart.setVelocity(KICK_SPEED);
      kickTimer = millis();
      lastZeroTime = millis();
      isKicking = true;
      Serial.println(F("GO !"));
    }
    if (c == 'e' || c == 'r') {
      cart.stop();
      digitalWrite(ENABLE_PIN, HIGH);
      state = WAITING;
      Serial.println(F("Stop."));
    }
  }

  // Logique principale
  if (state == RUNNING) {
    float theta = pendulumEnc.getAngleRad();
    
    // 1. Est-ce qu'on est en haut ?
    if (abs(abs(theta) - PI) < UPRIGHT_THRESHOLD) {
      cart.stop();
      digitalWrite(ENABLE_PIN, HIGH);
      state = DONE;
      Serial.println(F(">>> EN HAUT ! Moteur coupe. <<<"));
      return;
    }

    // 2. Gestion de la fin du coup en cours
    if (isKicking && (millis() - kickTimer > KICK_DURATION)) {
      cart.setVelocity(0.0f); // On arrête net le chariot après 200ms
      isKicking = false;
    }

    // 3. Détection du passage par ZÉRO (Avec Anti-bruit)
    bool isPositive = (theta >= 0.0f);
    
    // Si on change de signe ET qu'on est en bas
    if (isPositive != wasPositive && abs(theta) < PI/2) {
      
      // ---> LE VERROU ANTI-BRUIT EST ICI <---
      // On n'accepte de donner un coup QUE si le moteur a fini le précédent
      // ET qu'il s'est passé au moins 300 ms depuis le dernier passage à zéro
      if (!isKicking && (millis() - lastZeroTime > 300)) {
          
          float omega = pendulumEnc.getAngularVelocityRad();
          
          // On donne un coup dans la direction OPPOSÉE au pendule
          float velCmd = (omega > 0) ? -KICK_SPEED : KICK_SPEED;
          
          cart.setVelocity(velCmd);
          
          // On lance les chronos
          kickTimer = millis();
          lastZeroTime = millis();
          isKicking = true;
          
          Serial.println(F("Zero detecte, oppa !"));
      }
    }
    
    wasPositive = isPositive;
  }
}
