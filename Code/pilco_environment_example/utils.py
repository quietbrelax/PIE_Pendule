# -*- coding: utf-8 -*-
"""
Created on Fri Feb 14 14:46:11 2025

@author: davie
"""

"""
Fichier utils.py

Contient des fonctions utilitaires pouvant être utilisées dans le projet PILCO.
Parmi elles, on trouve par exemple le noyau RBF pour l'utilisation dans les modèles GP
et des fonctions de normalisation des données.
"""

import numpy as np

def rbf_kernel(X1, X2, lengthscale=1.0, variance=1.0):
    """
    Calcule le noyau RBF (Radial Basis Function) entre deux ensembles de points.
    
    :param X1: Array de dimension (n_samples1, n_features).
    :param X2: Array de dimension (n_samples2, n_features).
    :param lengthscale: Paramètre d'échelle (lengthscale) du noyau.
    :param variance: Variance du noyau.
    :return: Matrice de covariance de dimension (n_samples1, n_samples2).
    """
    # Calcul des distances au carré entre chaque couple de points
    sqdist = np.sum(X1**2, axis=1).reshape(-1, 1) + \
             np.sum(X2**2, axis=1) - 2 * np.dot(X1, X2.T)
    return variance * np.exp(-0.5 * sqdist / (lengthscale ** 2))

def normalize_data(X):
    """
    Normalise les données pour que chaque caractéristique ait une moyenne nulle et une variance unitaire.
    
    :param X: Array de dimension (n_samples, n_features).
    :return: Tuple (X_norm, mean, std) où X_norm est l'array normalisé.
    """
    mean = np.mean(X, axis=0)
    std = np.std(X, axis=0) + 1e-8  # ajout d'une petite valeur pour éviter la division par zéro
    X_norm = (X - mean) / std
    return X_norm, mean, std

def unnormalize_data(X_norm, mean, std):
    """
    Annule la normalisation d'un jeu de données.
    
    :param X_norm: Données normalisées.
    :param mean: Moyenne utilisée pour la normalisation.
    :param std: Écart-type utilisé pour la normalisation.
    :return: Données dans leur échelle originale.
    """
    return X_norm * std + mean
