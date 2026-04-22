## Structure du dossier

Le dossier est organisé en plusieurs sous dossiers :

### 1. **`simulation`**

Ces codes ont été effectués par l'équipe 2024/2025, nous avons refait les notres, mais il semble toujours utile de les laisser.

- **`Sim IHM V2.1.py`**  
  La simulation principale utilisant une intégration de **Runge-Kutta d'ordre 4**.  
  Cette simulation génère des résultats fluides du comportement d’un double pendule monté sur un chariot.
- **`methode_lqr.py`**  
  Une implémentation de la méthode **LQR (Linear Quadratic Regulator)**.  
  Ce script stabilise le double pendule en position verticale en linéarisant les équations autour de la position d’équilibre.

### 2. **`pilco environment example`**

Nous avons au final décidé de ne pas suivre la piste de PILCO donc ce dossier ne nous est plus utile, mais si vous êtes curieux ou voulez vous pencher dessus, vous retrouverez ici ce qu'il faut.

- Ce dossier contient un exemple d’environnement pour appliquer l'algorithme **PILCO** (Probabilistic Inference for Learning Control).  
  L’objectif est d’adapter cet algorithme au contrôle du double pendule, mais le projet est encore en cours de développement et **n'est pas encore fonctionnel**.

Principaux fichiers :

- `environment.py` : Définit l'environnement personnalisé pour PILCO.
- `pilco.py` : Contient l'implémentation principale de l'algorithme PILCO.
- `policy.py` et `utils.py` : Modules d'outils complémentaires.

### 3. **`simulation_simple_pendule`**

Simulation d'un pendule simple pour tester des méthodes de contrôle avant application au double pendule.

- **`pendule_simple.py`** : Simulation de base d'un pendule simple avec intégration RK4.
- **`pendule_simple_lqr.py`** : Contrôle LQR appliqué au pendule simple.
- **`swing_up.py`** : Algorithme de swing-up pour faire monter le pendule vers la position verticale.
- **`methode_lqr.py`** : Implémentation de la méthode LQR.
- **`Sim_IHM_V2_1.py`** : Interface graphique pour visualiser la simulation.

### 4. **`pendule_inverse_arduino`**

Code embarqué pour contrôler un pendule inverse réel sur hardware Arduino/ESP32.

- **`pendule_inverse.ino`** : Programme principal implémentant le contrôleur LQR en temps réel.
- **`encoder.h` / `encoder.cpp`** : Gestion du codeur incrémental pour mesurer l'angle du pendule.
- **`stepper_velocity.h` / `stepper_velocity.cpp`** : Contrôle du moteur pas à pas (driver A4988/DRV8825/TMC2209).

### 5. **`pilco_environment_example`** (En développement)

Exemple d'environnement pour tester l'algorithme **PILCO** (Probabilistic Inference for Learning Control).

⚠️ **Note** : Cette approche a été déprioritisée. Ces fichiers restent à titre de référence.

- `environment.py` : Environnement personnalisé PILCO
- `pilco.py` : Implémentation de l'algorithme
- `policy.py` et `utils.py` : Modules utilitaires

### 6. **`APM_4AUT2_TA_TP4_genet.ipynb`**

Notebook Jupyter contenant des travaux pratiques d'automatique, incluant des approches par algorithmes génétiques et optimisation.

---

## Guide pour démarrer

### Simulation Python (recommandé pour débuter)

1. **Installer les dépendances** :

   ```bash
   cd /Code
   pip install -r requirements.txt  # ou installez manuellement numpy, matplotlib, scipy
   ```

2. **Lancer une simulation simple** :

   ```bash
   python simulation_simple_pendule/pendule_simple.py
   ```

3. **Tester le contrôleur LQR** :

   ```bash
   python simulation_simple_pendule/pendule_simple_lqr.py
   ```
