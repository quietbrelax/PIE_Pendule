# -*- coding: utf-8 -*-
"""
Created on Fri Feb 14 14:46:00 2025

@author: davie
"""

"""
Fichier pilco.py

Contient l'implémentation de l'algorithme PILCO.
La classe PILCO orchestre la collecte de données, l'entraînement des modèles GP (ici simulé),
la propagation du coût et l'optimisation de la politique.
"""

import numpy as np
from utils import rbf_kernel  # exemple d'utilisation d'une fonction utilitaire

class PILCO:
    def __init__(self, env, policy):
        """
        Initialise l'instance PILCO avec l'environnement et la politique.
        
        :param env: Instance de l'environnement (ex: DoublePendulumCartEnv).
        :param policy: Instance de la politique (ex: Policy).
        """
        self.env = env
        self.policy = policy

        # Initialisation des jeux de données pour l'apprentissage du modèle de dynamique.
        # Ils stockeront respectivement : [x, u] et [x_next - x]
        self.X = None  # données d'entrée
        self.Y = None  # différences d'état

    def initial_random_rollouts(self, num_rollouts, rollout_length):
        """
        Effectue une collecte initiale de données via des rollouts aléatoires.
        
        :param num_rollouts: Nombre de rollouts à effectuer.
        :param rollout_length: Longueur de chaque rollout.
        """
        X_list = []
        Y_list = []

        print("Collecte de données initiale via des rollouts aléatoires...")
        for rollout in range(num_rollouts):
            state = self.env.reset()
            for t in range(rollout_length):
                # Action aléatoire (pour explorer l'espace d'états)
                action = np.random.randn(self.env.action_dim)
                next_state, cost, done, _ = self.env.step(action)
                # Stockage des entrées [state, action] et de la variation d'état
                X_list.append(np.hstack((state, action)))
                Y_list.append(next_state - state)
                state = next_state
        self.X = np.array(X_list)
        self.Y = np.array(Y_list)
        print("Données collectées :", self.X.shape, self.Y.shape)

    def train_gp_models(self):
        """
        "Entraîne" les modèles GP à partir des données collectées.
        Dans cet exemple, la fonction se contente d'une simulation d'entraînement.
        Dans une implémentation réelle, ici vous ajusteriez vos modèles GP (par exemple avec GPy).
        """
        print("Entraînement des modèles GP sur les données collectées...")
        # Code réel d'entraînement d'un modèle GP irait ici.
        # Par exemple : self.gp_model.fit(self.X, self.Y)
        # Pour cet exemple, nous simulons simplement un temps d'entraînement.
        pass

    def rollout(self, policy_params=None, rollout_length=50):
        """
        Effectue un rollout en utilisant la politique actuelle.
        Les nouvelles données obtenues sont ajoutées au jeu de données pour ajuster les GP.
        
        :param policy_params: Paramètres optionnels de la politique à appliquer.
        :param rollout_length: Nombre d'étapes du rollout.
        :return: (trajectory, total_cost) où trajectory est la liste des états et total_cost le coût cumulé.
        """
        # Si des paramètres sont fournis, on met à jour la politique.
        if policy_params is not None:
            self.policy.set_params(policy_params)

        state = self.env.reset()
        trajectory = [state.copy()]
        total_cost = 0.0

        for t in range(rollout_length):
            action = self.policy.act(state)
            next_state, cost, done, _ = self.env.step(action)
            # Ajout des nouvelles données au dataset :
            new_X = np.hstack((state, action))[None, :]  # dimensions (1, state_dim+action_dim)
            new_Y = (next_state - state)[None, :]         # dimensions (1, state_dim)
            if self.X is None:
                self.X = new_X
                self.Y = new_Y
            else:
                self.X = np.vstack((self.X, new_X))
                self.Y = np.vstack((self.Y, new_Y))
            trajectory.append(next_state.copy())
            total_cost += cost
            state = next_state

        return trajectory, total_cost

    def run(self, iterations, rollout_length):
        """
        Exécute la boucle d'apprentissage PILCO :
          - Rollout avec la politique actuelle
          - Mise à jour des modèles GP
          - Optimisation de la politique via un pas de gradient simulé
        
        :param iterations: Nombre d'itérations d'apprentissage.
        :param rollout_length: Longueur de chaque rollout.
        """
        for itr in range(iterations):
            print("\nItération", itr + 1, "sur", iterations)
            trajectory, rollout_cost = self.rollout(rollout_length=rollout_length)
            print("Coût du rollout :", rollout_cost)

            # Optimisation simulée de la politique (exemple avec un gradient aléatoire)
            dummy_gradient = np.random.randn(*self.policy.K.shape) * 0.01
            self.policy.update(dummy_gradient, learning_rate=1e-3)
            print("Mise à jour de la politique (dummy update).")

            # Réentraîner les modèles GP avec les données enrichies par le rollout
            self.train_gp_models()
