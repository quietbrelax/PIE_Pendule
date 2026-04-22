# -*- coding: utf-8 -*-
"""
Created on Fri Feb 14 14:39:52 2025

@author: davie
"""

"""
Fichier environment.py

Contient l'implémentation de l'environnement de simulation du double pendule sur chariot.
Inclut la dynamique du système et une méthode d'intégration via Runge-Kutta 4.
"""

import numpy as np

class DoublePendulumCartEnv:
    def __init__(self, dt=0.01):
        """
        Initialise l'environnement avec les paramètres physiques et numériques.
        
        :param dt: Pas de temps pour l'intégration numérique.
        """
        self.dt = dt
        # Dimensions de l'état et de l'action en fonction du problème.
        # Exemple d'état : [position_du_chariot, vitesse_du_chariot, theta1, theta1_dot, theta2, theta2_dot]
        self.state_dim = 6  
        self.action_dim = 1   # Exemple : force appliquée sur le chariot

        # Paramètres physiques (exemples, à adapter)
        self.mass_cart = 1.0
        self.mass_pend1 = 0.1
        self.mass_pend2 = 0.1
        self.length1 = 1.0
        self.length2 = 1.0
        self.gravity = 9.81

        self.reset()

    def reset(self):
        """
        Réinitialise l'environnement en assignant un état initial.

        :return: Nouvel état initial.
        """
        # Initialisation de l'état en partant d'un équilibre légèrement perturbé pour éviter la symétrie parfaite.
        self.state = np.zeros(self.state_dim)
        self.state[2] = 0.1   # petite perturbation pour theta1
        self.state[4] = -0.1  # petite perturbation pour theta2
        return self.state

    def step(self, action):
        """
        Exécute une étape de simulation : intègre la dynamique et retourne le nouvel état,
        le coût associé, l'indicateur "done" et d'éventuelles infos complémentaires.
        
        :param action: Action appliquée (array de dimension action_dim).
        :return: (nouvel état, coût, done, info)
        """
        # Intégration de la dynamique via RK4
        state_next = self._rk4_integration(self.state, action)
        self.state = state_next.copy()

        # Calcul du coût (par exemple, minimiser la déviation par rapport à zéro et l'utilisation excessive de force)
        cost = self._cost(self.state, action)

        # Pour cet exemple, nous considérons que l'épisode ne se termine jamais (à adapter si besoin)
        done = False
        
        return self.state, cost, done, {}

    def _rk4_integration(self, state, action):
        """
        Utilise Runge-Kutta d'ordre 4 pour intégrer les équations différentielles du système.
        
        :param state: État courant.
        :param action: Action appliquée.
        :return: Nouvel état après intégration.
        """
        dt = self.dt

        def dynamics(s, a):
            """
            Définit les équations différentielles (dynamique simplifiée) du double pendule sur chariot.
            
            :param s: État du système [x, x_dot, theta1, theta1_dot, theta2, theta2_dot].
            :param a: Action appliquée.
            :return: Dérivée de l'état.
            """
            ds = np.zeros_like(s)
            x, x_dot, theta1, theta1_dot, theta2, theta2_dot = s

            # Dynamique du chariot
            ds[0] = x_dot
            ds[1] = a[0] / self.mass_cart  # Simplification : la force appliquée directement sur le chariot

            # Dynamique du pendule 1
            ds[2] = theta1_dot
            ds[3] = -self.gravity / self.length1 * np.sin(theta1)

            # Dynamique du pendule 2
            ds[4] = theta2_dot
            ds[5] = -self.gravity / self.length2 * np.sin(theta2)
            return ds

        # Intégration RK4
        k1 = dynamics(state, action)
        k2 = dynamics(state + dt * k1 / 2, action)
        k3 = dynamics(state + dt * k2 / 2, action)
        k4 = dynamics(state + dt * k3, action)

        state_next = state + dt * (k1 + 2*k2 + 2*k3 + k4) / 6
        return state_next

    def _cost(self, state, action):
        """
        Calcule le coût par rapport à l'état et à l'action.
        Ici, le coût est quadratique dans l'état et l'action.
        
        :param state: État du système.
        :param action: Action appliquée.
        :return: Coût scalaire.
        """
        cost_state = np.sum(state**2)
        cost_action = np.sum(action**2)
        return cost_state + cost_action
