# -*- coding: utf-8 -*-
"""
Created on Fri Feb 14 14:40:19 2025

@author: davie
"""

"""
Fichier policy.py

Contient la définition de la politique servant à déterminer l'action à partir de l'état observé.
Dans cet exemple, nous utilisons une politique linéaire simple: u = Kx.
"""

import numpy as np

class Policy:
    def __init__(self, state_dim, action_dim):
        """
        Initialise la politique avec des paramètres aléatoires.
        
        :param state_dim: Dimension de l'état.
        :param action_dim: Dimension de l'action.
        """
        self.state_dim = state_dim
        self.action_dim = action_dim
        
        # Politique linéaire : K est une matrice de dimension (action_dim x state_dim)
        self.K = np.random.randn(action_dim, state_dim) * 0.1

    def act(self, state):
        """
        Calcule l'action à réaliser pour un état donné.
        
        :param state: État du système (array de dimension state_dim).
        :return: Action (array de dimension action_dim).
        """
        action = self.K.dot(state)
        # Optionnel : limiter l'action avec np.clip(action, -max_action, max_action)
        return action

    def get_params(self):
        """
        Retourne les paramètres de la politique.
        
        :return: Matrice de gains K.
        """
        return self.K

    def set_params(self, K):
        """
        Met à jour les paramètres de la politique.
        
        :param K: Nouvelle matrice de gains.
        """
        self.K = K.copy()

    def update(self, gradient, learning_rate=1e-3):
        """
        Met à jour les paramètres via un simple pas de gradient.
        
        :param gradient: Gradient de même dimension que K.
        :param learning_rate: Taux d'apprentissage.
        """
        self.K -= learning_rate * gradient
