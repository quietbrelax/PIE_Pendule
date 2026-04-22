# -*- coding: utf-8 -*-
"""
Created on Fri Feb 14 14:39:23 2025

@author: davie
"""

"""
Fichier main.py

Point d'entrée du projet PILCO appliqué au double pendule sur chariot.
Ce script initialise l'environnement, la politique ainsi que l'algorithme PILCO,
effectue une phase de collecte de données initiale, lance la boucle d'apprentissage
et enfin teste la politique obtenue.
"""

from environment import DoublePendulumCartEnv
from policy import Policy
from pilco import PILCO

import numpy as np

def main():
    # Pour garantir la reproductibilité
    np.random.seed(42)

    # Initialisation de l'environnement
    dt = 0.01  # pas de temps utilisé dans la simulation
    env = DoublePendulumCartEnv(dt=dt)
    
    # Initialisation de la politique.
    # Les dimensions d'état et d'action sont définies dans l'environnement.
    policy = Policy(state_dim=env.state_dim, action_dim=env.action_dim)
    
    # Création de l'instance PILCO qui orchestre l'apprentissage par renforcement
    pilco_agent = PILCO(env, policy)

    # Phase de collecte de données initiale par rollouts aléatoires
    print("Collecte de données initiale...")
    pilco_agent.initial_random_rollouts(num_rollouts=3, rollout_length=50)

    # Entraînement initial des modèles GP avec ces données
    pilco_agent.train_gp_models()

    # Boucle d'apprentissage PILCO : collecte de données, mise à jour des GPs et optimisation de la politique
    print("Début de la boucle d'apprentissage PILCO...")
    pilco_agent.run(iterations=20, rollout_length=50)

    # Test de la politique finale sur un rollout prolongé
    print("Test de la politique obtenue...")
    traj, final_cost = pilco_agent.rollout(policy_params=policy.get_params(), rollout_length=100)
    print("Coût final du rollout test :", final_cost)

    # Optionnel : visualisation de la trajectoire (par exemple la position du chariot)
    try:
        import matplotlib.pyplot as plt
        traj = np.array(traj)
        plt.figure()
        plt.plot(traj[:, 0], label='Position du chariot')
        plt.xlabel("Pas de simulation")
        plt.ylabel("Position (m)")
        plt.title("Rollout final – Position du chariot")
        plt.legend()
        plt.show()
    except ImportError:
        print("Matplotlib n'est pas installé. La visualisation de la trajectoire est désactivée.")

if __name__ == "__main__":
    main()
