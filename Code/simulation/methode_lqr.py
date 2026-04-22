# -*- coding: utf-8 -*-
"""
Created on Tue Dec 24 17:03:18 2024

@author: Lucas Daviet
"""

"""
Simulation d'un double pendule inversé sur un chariot avec LQR
===============================================================
Ce programme simule et affiche graphiquement un double pendule inversé monté 
sur un chariot. La stabilisation est réalisée à l'aide d'un contrôleur optimal 
LQR (Linear Quadratic Regulator). Les équations du mouvement sont résolues 
numériquement avec la méthode de Runge-Kutta d'ordre 4.

Fonctionnalités principales :
- Contrôle manuel des perturbations (touches fléchées) ou automatique via LQR.
- Prise en compte des frottements visqueux et secs.
- Visualisation en temps réel avec Pygame.

Modules requis : numpy, pygame, scipy

-------------------------------------------------------------------------------

Simulation of a Double Inverted Pendulum on a Cart with LQR
============================================================
This program simulates and graphically displays a double inverted pendulum 
mounted on a cart. Stabilization is achieved using an optimal LQR 
(Linear Quadratic Regulator) controller. Motion equations are solved 
numerically using the 4th-order Runge-Kutta method.

Key features:
- Manual control (arrow keys) or automatic stabilization via LQR.
- Accounts for viscous and dry friction.
- Real-time visualization using Pygame.

Required modules: numpy, pygame, scipy
"""


import numpy as np
import pygame
#from scipy.signal import place_poles
from scipy.linalg import solve_continuous_are
#import time

# Définition des constantes physiques
g = 9.81  # Accélération gravitationnelle (m/s²)
m0 = 5  # Masse du chariot (kg)
m1 = 0.3  # Masse du premier pendule (kg)
m2 = 0.3  # Masse du deuxième pendule (kg)

L1 = 1  # Longueur totale du premier pendule (m)
L2 = 1  # Longueur totale du deuxième pendule (m)

# Calcul des distances au centre de masse
l1 = 1/2 * L1  # Distance entre la base et le centre de masse du premier pendule (m)
l2 = 1/2 * L2  # Distance entre la base et le centre de masse du deuxième pendule (m)

# Calcul des moments d'inertie
I1 = 1/12 * m1 * L1**2  # Moment d'inertie du premier pendule (kg·m²)
I2 = 1/12 * m2 * L2**2  # Moment d'inertie du deuxième pendule (kg·m²)

# Coefficients de frottement visqueux
b0 = 0.5  # Frottement sur le chariot
b1 = 0.025  # Frottement sur le premier pendule
b2 = 0.025  # Frottement sur le deuxième pendule

# Variables dynamiques
x = 0.0  # Position horizontale du chariot (m)
x_point = 0.0  # Vitesse du chariot (m/s)
x_point_point = 0.0 # Accélération du chariot (m/s2)

theta1 = 12 * ((2*np.pi)/360)  # Angle du premier pendule par rapport à la verticale (rad)
theta1_point = 0.0  # Vitesse angulaire du premier pendule (rad/s)
theta1_point_point = 0.0 # Accélération angulaire du premier pendule (rad/s2)

theta2 = -12 * ((2*np.pi)/360) # Angle du deuxième pendule par rapport à la verticale (rad)
theta2_point = 0.0  # Vitesse angulaire du deuxième pendule (rad/s)
theta2_point_point = 0.0 # Accélération angulaire du deuxième pendule (rad/s2)

# Force de contrôle
u = 0.0  # Force appliquée au chariot (N)

dt = 0.01  # Pas de temps
freq = 1/dt # Fréquence d'image

# Initialisation des états
initial_state = np.array([x, x_point, theta1, theta1_point, theta2, theta2_point])
# Initialisation de l'état d'équilibre recherché
state_eq = np.array([0,0,0])
state_point_eq = np.array([0,0,0])
# Affichage des paramètres physiques initiaux :
print(f"l1 = {l1}, I1 = {I1}")
print(f"l2 = {l2}, I2 = {I2}")
print("État initial :", initial_state)

# Couleurs
BLACK = (50, 50, 50)
WHITE = (255, 255, 255)
RED = (255, 50, 50)
ORANGE = (255, 165, 0)
GREEN = (125, 125, 0)
GRAY = (150, 150, 150)

def Conversion_MP(x) : # Prend une valeur en mètres, retourne une valeur en pixels
    return x*100 # 1m = 100 pixels

# Paramètres visuels
chariot_width = 50
chariot_height = 20
wheel_radius = 9 # Uniquement pour le style
pendulum_length1 = Conversion_MP(L1)  # Longueur visuelle du premier pendule (pixels)
pendulum_length2 = Conversion_MP(L1)  # Longueur visuelle du deuxième pendule (pixels)
scale = 1  # Échelle pour ajuster le rendu

# Initialisation de Pygame
pygame.init()

# Dimensions de la fenêtre
WIDTH, HEIGHT = 2000, 1000
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Double Pendule sur Chariot V2.0")

# Centre de la scène
pivot_x = WIDTH // 2
pivot_y = HEIGHT // 2

# Vecteurs d'état :
state = np.array([x, theta1, theta2])
state_point = np.array([x_point, theta1_point, theta2_point])
state_point_point = np.array([x_point_point, theta1_point_point, theta2_point_point])

########---------------LQR----------------#######

# Initialisation Matrices de coût 

# Q est un matrice diagonale où chaque coef représente une pénalité liée à une coordonnée de notre système
# Si Q[0,0] = 0.00001 alors x va atteindre sa position d'équilibre de manière très lente
# Si Q[0,0] = 100000 alors x va atteindre sa position d'équilibre de manière très brutale (pouvant mener à se que le système se brise et diverge)
Q = np.array([[1,0,0,0,0,0],[0,100,0,0,0,0],[0,0,100,0,0,0],[0,0,0,1,0,0],[0,0,0,0,1,0],[0,0,0,0,0,1]])

# R pénalise la force appliquée au chariot u
# si R vaut 0.00001 alors la force ne sera pas bornée et libre d'atteindre les valeurs qu'elle veut
# Si R vaut 1000000 alors la force sera bornée et évitera qu'elle aille dans les très valeurs
# Cependant si R est trop et qu'on lui met une perturbation (coup de règle sur le pendule une fois l'équilibre atteint) il se peut que le pendule diverge
R=0.00001


# Lqr c'est la résolution de calculs pour trouver K tel que u = - K*x
def lqr(A, B, Q, R):
    """
    Résout le problème LQR pour le système linéaire continu :
    dx/dt = Ax + Bu
    
    Args:
        A : Matrice système (n x n)
        B : Matrice de commande (n x m)
        Q : Matrice pondérant l'état (n x n)
        R : Matrice pondérant le contrôle (m x m)
    
    Returns:
        K : Gain de retour d'état optimal (m x n)
        S : Matrice de Riccati solution
        eigVals : Valeurs propres du système fermé
    """
    # Résout l'équation de Riccati algébrique continue
    S = solve_continuous_are(A, B, Q, R)
    
    # Calcul du gain optimal K
    K = (1/R)*((B.T)@(S))
    
    # Valeurs propres du système fermé
    eigVals = np.linalg.eig(A - B @ K)[0]
    return K

##############-------Calcul de K ---------############
def Calc_K(Q , R  ):


#############################Linéarisation des équations pour tous les cas d'eq possibles (à noter que C = 0 tout le temps) ##################################################
######################################################################################################################################################################################
#######################################################################################################################################################################################



#################################### theta 1 = theta 2 = 0 ############################################################

################# cf la thèse et les équations indiquées ########################### 
#####################################################################################
#####################################################################################



    NEWD = np.array([ [m0+m1+m2,   (1/2*m1+m2)*L1,   1/2*m2*L2]
                  ,[(1/2*m1+m2)*L1,     (1/3*m1+m2)*L1*L1,      1/2*m2*L1*L2]
                  ,[1/2*m2*L2,      1/2*m2*L1*L2,     1/3*m2*L2*L2]])
    INVD = np.linalg.inv(NEWD)


    NEWG = np.array([[0,0,0]
                     ,[0,-L1*g/2*(m1+2*m2),0]
                     ,[0,0,-L2*g/2*m2]])
    

############################ Les 3 premières colones de A ###################
    INTERA =  np.vstack((np.zeros((3,3)),-np.dot(INVD,NEWG)))


############################ Les 3 dernières colones de A ####################
    INTERA2 = np.vstack( ( np.eye(3),np.zeros((3,3)) ) ) - np.array([[0,0,0]
                                                                    ,[0,0,0]
                                                                    ,[0,0,0]
                                                                    ,[b0,0,0]
                                                                    ,[0,b1,0]
                                                                    ,[0,0,b2]])

######################## Calcul de A et B ######################################
    A = np.hstack((INTERA, INTERA2))
    B = np.array([0,0,0,INVD[0,0],INVD[1,0],INVD[2,0]]).reshape((6,1))

    K = lqr(A,B,Q,R) # On obtient notre matrice de gain de taille (1,6)
    return K 

# on calcule K 
K = Calc_K(Q,R)

######----------- Fin du LQR ------------------------------######

def Calc_D(state):
    """
    Calcule la matrice D(theta) pour le double pendule inversé.
    (Matrice d'intertie, Cf document support pour les équations du mouvement. )

    Paramètres :
        theta1 : Angle du premier pendule (rad)
        theta2 : Angle du deuxième pendule (rad)

    Retourne :
        Une matrice numpy 3x3 représentant D(theta).
    """
    # Extraction des positions :
    theta1 = state[1]
    theta2 = state[2]
    
    # Calcul des éléments de la matrice
    D11 = m0 + m1 + m2
    D12 = (0.5 * m1 + m2) * L1 * np.cos(theta1)
    D13 = 0.5 * m2 * L2 * np.cos(theta2)

    D21 = (0.5 * m1 + m2) * L1 * np.cos(theta1)
    D22 = ((1/3)*m1+m2)*L1**2
    D23 = 0.5 * m2 * L1 * L2 * np.cos(theta1 - theta2)

    D31 = 0.5 * m2 * L2 * np.cos(theta2)
    D32 = 0.5 * m2 * L1 * L2 * np.cos(theta1 - theta2)
    D33 = (1/3) * m2 * L2**2

    # Construction de la matrice
    D_matrix = np.array([
        [D11, D12, D13],
        [D21, D22, D23],
        [D31, D32, D33]
    ])

    return D_matrix

def Calc_C(state, state_point):
    """
    Calcule la matrice C(theta, theta_point) pour le double pendule inversé.
    (Matrice des termes de Coriolis, Cf document support pour les équations du mouvement.)
    
    Paramètres :
        theta1 : Angle du premier pendule (rad)
        theta1_point : Vitesse angulaire du premier pendule (rad/s)
        theta2 : Angle du deuxième pendule (rad)
        theta2_point : Vitesse angulaire du deuxième pendule (rad/s)

    Retourne :
        Une matrice numpy 3x3 représentant C(theta, theta_point).
    """
    # Extraction des positions et vitesse
    theta1 = state[1]
    theta2 = state[2]
    theta1_point = state_point[1]
    theta2_point = state_point[2]

    # Calcul des éléments de la matrice
    C11 = 0
    C12 = -(0.5 * m1 + m2) * L1 * np.sin(theta1) * theta1_point
    C13 = -0.5 * m2 * L2 * np.sin(theta2) * theta2_point

    C21 = 0
    C22 = 0
    C23 = 0.5 * m2 * L1 * L2 * np.sin(theta1 - theta2) * theta2_point

    C31 = 0
    C32 = -0.5 * m2 * L1 * L2 * np.sin(theta1 - theta2) * theta1_point
    C33 = 0

    # Construction de la matrice
    C_matrix = np.array([
        [C11, C12, C13],
        [C21, C22, C23],
        [C31, C32, C33]
    ])

    return C_matrix

def Calc_G(state):
    """
    Calcule le vecteur G(theta) pour le double pendule inversé.
    (Vecteur des termes gravitationnels, Cf document support pour les équations du mouvement.)

    Paramètres :
        theta1 : Angle du premier pendule (rad)
        theta2 : Angle du deuxième pendule (rad)

    Retourne :
        Un vecteur numpy 3x1 représentant G(theta).
    """
    # Extraction des positions :
    theta1 = state[1]
    theta2 = state[2]

    # Calcul des composantes du vecteur
    G1 = 0
    G2 = -0.5 * (m1 + m2) * L1 * g * np.sin(theta1)
    G3 = -0.5 * m2 * L2 * g * np.sin(theta2)

    # Construction du vecteur
    G_vector = np.array([G1, G2, G3])

    return G_vector

H = np.array([1,0,0]) # Quatrième vecteur des équations du mouvement 
# Représente la contribution de la force u sur le chariot.

# Frottement sec
def calc_friction_dry(state_point):
    bonus  = 0
    return np.array([
        -np.sign(state_point[0]) * 0.1*bonus,  # Frottement sec sur le chariot
        -np.sign(state_point[1]) * 0.0125*bonus,  # Frottement sec sur le premier pendule
        -np.sign(state_point[2]) * 0.0125*bonus   # Frottement sec sur le deuxième pendule
    ])

def modulo(theta):
    theta = np.sign(theta)* ((abs(theta))%(2*np.pi))
    if theta > np.pi:
        theta = theta % (np.pi*2)
        return  theta - 2*np.pi
    elif theta < -np.pi:
        return 2*np.pi + theta
    return theta


def Calc_u(K,state,state_point,state_eq,state_point_eq):
    full_state = np.hstack((state-state_eq,state_point-state_point_eq))
    full_state = full_state.reshape((6,1))
    u = -np.dot(K,full_state)[0]
    #print("La force : " , u ," L'erreur : " , full_state.T )
    return u

def Calc_accelerations(state, state_point, u,fu) :
    # Renvoie le vecteur accélération state_point_point, avec frottements visqueux
    D = Calc_D(state)
    C = Calc_C(state, state_point)
    G = Calc_G(state)
    D_inv = np.linalg.inv(D)

    # Forces de frottement
    friction = np.array([
        -b0 * state_point[0],  # Frottement sur le chariot
        -b1 * state_point[1],  # Frottement sur le premier pendule
        -b2 * state_point[2]   # Frottement sur le deuxième pendule
    ])

    friction_dry = calc_friction_dry(state_point)
    state_point_point = (-np.dot(D_inv, np.dot(C, state_point)) 
                     - np.dot(D_inv, G) 
                     + np.dot(D_inv, H * (u+fu)) 
                     + np.dot(D_inv, friction) 
                     + np.dot(D_inv, friction_dry))
    return state_point_point

def runge_kutta_step(state, state_point, state_eq, state_point_eq, K,fu):
    """
    Effectue un pas de Runge-Kutta 4 pour mettre à jour l'état du système.

    Paramètres :
        state : Vecteur des positions actuelles [x, theta1, theta2].
        state_point : Vecteur des vitesses actuelles [x_point, theta1_point, theta2_point].
        u : Force appliquée au chariot (N).

    Retourne :
        (new_state, new_state_point) : Les nouvelles positions et vitesses après un pas de temps.
    """

    u = Calc_u(K,state,state_point,state_eq,state_point_eq)
    # Première étape : k1
    k1_state = state_point
    k1_state_point = Calc_accelerations(state, state_point, u,fu)

    # Deuxième étape : k2
    state2 = state + 0.5 * dt * k1_state
    state_point2 = state_point + 0.5 * dt * k1_state_point
    u = Calc_u(K,state2,state_point2,state_eq,state_point_eq)
    k2_state = state_point2
    k2_state_point = Calc_accelerations(state2, state_point2, u,fu)

    # Troisième étape : k3
    state3 = state + 0.5 * dt * k2_state
    state_point3 = state_point + 0.5 * dt * k2_state_point
    u = Calc_u(K,state3,state_point3,state_eq,state_point_eq)
    k3_state = state_point3
    k3_state_point = Calc_accelerations(state3, state_point3, u,fu)

    # Quatrième étape : k4
    state4 = state + dt * k3_state
    state_point4 = state_point + dt * k3_state_point
    u = Calc_u(K,state4,state_point4,state_eq,state_point_eq)
    k4_state = state_point4
    k4_state_point = Calc_accelerations(state4, state_point4, u,fu)

    # Combinaison des termes de Runge-Kutta
    new_state = state + (dt / 6) * (k1_state + 2 * k2_state + 2 * k3_state + k4_state)
    new_state_point = state_point + (dt / 6) * (k1_state_point + 2 * k2_state_point + 2 * k3_state_point + k4_state_point)

    return new_state, new_state_point

def Afficher(state):
    """
    Affiche le double pendule et le chariot dans la fenêtre Pygame.

    Paramètres :
        state : Vecteur des positions actuelles [x, theta1, theta2].
    """

    # Extraction des positions
    x = state[0]  # Position horizontale du chariot
    theta1 = state[1]  # Angle du premier pendule
    theta2 = state[2]  # Angle du deuxième pendule

    # Calcul des positions absolues
    chariot_x = pivot_x + Conversion_MP(x)  # Position du chariot (en pixels)
    pendulum1_x = chariot_x + pendulum_length1 * np.sin(theta1)
    pendulum1_y = pivot_y - pendulum_length1 * np.cos(theta1)
    pendulum2_x = pendulum1_x + pendulum_length2 * np.sin(theta2)
    pendulum2_y = pendulum1_y - pendulum_length2 * np.cos(theta2)

    # Effacement de l'écran
    screen.fill(BLACK)

    # Dessin des rails
    pygame.draw.line(screen, GRAY, (0, pivot_y + chariot_height // 2 + wheel_radius),
                     (WIDTH, pivot_y + chariot_height // 2 + wheel_radius), 4)

    # Dessin du chariot
    pygame.draw.rect(screen, GREEN, 
                     (chariot_x - chariot_width // 2, pivot_y, chariot_width, chariot_height))

    # Dessin des roues
    pygame.draw.circle(screen, WHITE, 
                       (chariot_x - chariot_width // 3, pivot_y + chariot_height // 2 + wheel_radius), wheel_radius)
    pygame.draw.circle(screen, WHITE, 
                       (chariot_x + chariot_width // 3, pivot_y + chariot_height // 2 + wheel_radius), wheel_radius)

    # Dessin du premier pendule
    pygame.draw.line(screen, WHITE, (chariot_x, pivot_y), (pendulum1_x, pendulum1_y), 4)
    pygame.draw.circle(screen, ORANGE, (int(pendulum1_x), int(pendulum1_y)), 10)

    # Dessin du deuxième pendule
    pygame.draw.line(screen, WHITE, (pendulum1_x, pendulum1_y), (pendulum2_x, pendulum2_y), 4)
    pygame.draw.circle(screen, RED, (int(pendulum2_x), int(pendulum2_y)), 10)

    # Mise à jour de l'écran
    pygame.display.flip()


# Boucle principale
running = True
clock = pygame.time.Clock()

fu=0
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Contrôle avec le clavier
    keys = pygame.key.get_pressed()
    if keys[pygame.K_LEFT]:
        fu+= -20
    elif keys[pygame.K_RIGHT]:
        fu += 20.0
    # else:
    #     u = 0.0

    # Mise à jour des états (Runge-Kutta)
    state, state_point = runge_kutta_step(state, state_point, state_eq, state_point_eq, K,fu)

    # Affichage du système
    Afficher(state)

    # Limitation de la fréquence d'affichage
    clock.tick(freq)

pygame.quit()



