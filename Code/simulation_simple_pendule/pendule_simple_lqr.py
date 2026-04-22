# -*- coding: utf-8 -*-
"""
Simulation d'un pendule simple inversé sur un chariot avec LQR
==============================================================
Pendule simple sur chariot avec résolution RK4 et stabilisation LQR.
Contrôle : touches ← → pour appliquer une perturbation au chariot.

Modules requis : numpy, pygame, scipy
"""

import numpy as np
import pygame
from scipy.linalg import solve_continuous_are

# --- Paramètres physiques ---
g   = 9.81   # Accélération gravitationnelle (m/s²)
m0  = 5.0    # Masse du chariot (kg)
m1  = 0.3    # Masse du pendule (kg)
L1  = 1.0    # Longueur totale du pendule (m)
l1  = L1 / 2 # Distance axe → centre de masse (m)
I1  = (1/12) * m1 * L1**2  # Moment d'inertie (kg·m²)

# Frottements visqueux
b0 = 0.5    # Chariot
b1 = 0.025  # Pendule

# Frottements secs
fs0 = 0.1    # Chariot
fs1 = 0.0125 # Pendule

dt   = 0.01        # Pas de temps (s)
freq = int(1 / dt) # Fréquence d'affichage (Hz)

print(f"l1 = {l1:.3f} m,  I1 = {I1:.4f} kg·m²")

# --- État initial : pendule légèrement incliné pour voir le LQR agir ---
# theta = 0 correspond à la position haute (équilibre instable)
state       = np.array([0.0, 0.1])     # [x, theta] 
state_point = np.array([0.0, 0.0])     # [x_dot, theta_dot]

# État d'équilibre visé
state_eq    = np.array([0.0, 0.0])


# -----------------------------------------------------------------------
# LQR : Calcul du gain K par linéarisation autour de l'équilibre
# -----------------------------------------------------------------------

# Q : Pénalités sur les états [x, theta, x_dot, theta_dot]
Q = np.diag([10, 100, 1, 1])

# R : Pénalité sur l'effort de commande u
R = np.array([[0.01]])

def lqr(A, B, Q, R):
    S = solve_continuous_are(A, B, Q, R)
    K = (1/R) * (B.T @ S)
    return K

def Calc_K(Q, R):
    # Linéarisation pour theta = 0 (position haute)
    # Matrice de masse évaluée à theta = 0
    Deq = np.array([
        [m0 + m1,       0.5 * m1 * L1],
        [0.5 * m1 * L1, (1/3) * m1 * L1**2]
    ])
    D_inv = np.linalg.inv(Deq)

    # Matrice de rigidité (issue du terme gravitationnel G linéarisé)
    M_pos = np.array([
        [0, 0],
        [0, 0.5 * m1 * L1 * g]
    ])

    # Matrice d'amortissement (frottements visqueux)
    M_vel = np.array([
        [-b0, 0],
        [0,  -b1]
    ])

    # Vecteur d'application de la force
    H_vec = np.array([[1], [0]])

    # Construction des matrices d'état A et B pour X_dot = A*X + B*u
    A21 = D_inv @ M_pos
    A22 = D_inv @ M_vel
    B2  = D_inv @ H_vec

    A = np.block([
        [np.zeros((2, 2)), np.eye(2)],
        [A21, A22]
    ])
    B = np.vstack((np.zeros((2, 1)), B2))

    return lqr(A, B, Q, R)

K = Calc_K(Q, R)
print("\nGain LQR calculé (K) :")
print(np.round(K, 3))


# -----------------------------------------------------------------------
# Physique : matrices non-linéaires D, C, G 
# -----------------------------------------------------------------------

def Calc_D(state):
    th = state[1]
    return np.array([
        [m0 + m1,                  (0.5*m1)*L1*np.cos(th)],
        [(0.5*m1)*L1*np.cos(th),   (1/3)*m1*L1**2        ]
    ])

def Calc_C(state, state_point):
    th  = state[1]
    thd = state_point[1]
    return np.array([
        [0, -(0.5*m1)*L1*np.sin(th)*thd],
        [0,  0                          ]
    ])

def Calc_G(state):
    th = state[1]
    return np.array([
        0,
        -0.5 * m1 * L1 * g * np.sin(th)
    ])

H = np.array([1.0, 0.0])

def calc_friction(state_point):
    return np.array([
        -b0 * state_point[0] - fs0 * np.sign(state_point[0]),
        -b1 * state_point[1] - fs1 * np.sign(state_point[1])
    ])

def Calc_accelerations(state, state_point, u, fu):
    D     = Calc_D(state)
    C     = Calc_C(state, state_point)
    G     = Calc_G(state)
    D_inv = np.linalg.inv(D)
    f     = calc_friction(state_point)

    acc = (- D_inv @ (C @ state_point)
           - D_inv @ G
           + D_inv @ (H * (u + fu))  # u est la force LQR, fu est la perturbation
           + D_inv @ f)
    return acc

def Calc_u(state, state_point):
    # On ramène l'angle entre -pi et pi pour éviter que l'erreur s'accumule si le pendule fait des tours complets
    err_th = (state[1] - state_eq[1] + np.pi) % (2 * np.pi) - np.pi
    
    X_err = np.array([
        state[0] - state_eq[0],
        err_th,
        state_point[0],
        state_point[1]
    ])
    return -(K @ X_err)[0]


# -----------------------------------------------------------------------
# Intégrateur Runge-Kutta 4
# -----------------------------------------------------------------------

def runge_kutta_step(state, state_point, fu):
    # Étape 1
    u1  = Calc_u(state, state_point)
    k1s = state_point
    k1d = Calc_accelerations(state, state_point, u1, fu)

    # Étape 2
    s2  = state       + 0.5*dt*k1s
    sd2 = state_point + 0.5*dt*k1d
    u2  = Calc_u(s2, sd2)
    k2s = sd2
    k2d = Calc_accelerations(s2, sd2, u2, fu)

    # Étape 3
    s3  = state       + 0.5*dt*k2s
    sd3 = state_point + 0.5*dt*k2d
    u3  = Calc_u(s3, sd3)
    k3s = sd3
    k3d = Calc_accelerations(s3, sd3, u3, fu)

    # Étape 4
    s4  = state       + dt*k3s
    sd4 = state_point + dt*k3d
    u4  = Calc_u(s4, sd4)
    k4s = sd4
    k4d = Calc_accelerations(s4, sd4, u4, fu)

    new_state       = state       + (dt/6)*(k1s + 2*k2s + 2*k3s + k4s)
    new_state_point = state_point + (dt/6)*(k1d + 2*k2d + 2*k3d + k4d)
    return new_state, new_state_point


# -----------------------------------------------------------------------
# Affichage Pygame
# -----------------------------------------------------------------------

def Conversion_MP(val):
    return val * 100

pygame.init()
WIDTH, HEIGHT = 1400, 700
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Pendule Simple LQR")
clock = pygame.time.Clock()
font  = pygame.font.SysFont("monospace", 18)

# Couleurs
BG     = ( 50,  50,  50)
WHITE  = (255, 255, 255)
ORANGE = (255, 165,   0)
GREEN  = (125, 125,   0)
GRAY   = (150, 150, 150)
CYAN   = (  0, 200, 220)
RED    = (255,  50,  50)

pivot_x = WIDTH  // 2
pivot_y = HEIGHT // 2
chariot_w, chariot_h = 60, 24
wheel_r = 9
pend_px = Conversion_MP(L1)


def Afficher(state, state_point, u, fu):
    x  = state[0]
    th = state[1]

    cx = int(pivot_x + Conversion_MP(x))
    cy = pivot_y

    px = int(cx + pend_px * np.sin(th))
    py = int(cy - pend_px * np.cos(th))

    screen.fill(BG)

    # Rail
    pygame.draw.line(screen, GRAY,
                     (0,     cy + chariot_h//2 + wheel_r),
                     (WIDTH, cy + chariot_h//2 + wheel_r), 4)

    # Chariot
    pygame.draw.rect(screen, GREEN,
                     (cx - chariot_w//2, cy - chariot_h//2, chariot_w, chariot_h))

    # Roues
    for wx in [cx - chariot_w//3, cx + chariot_w//3]:
        pygame.draw.circle(screen, WHITE,
                           (wx, cy + chariot_h//2 + wheel_r), wheel_r)

    # Pendule
    pygame.draw.line(screen, WHITE, (cx, cy), (px, py), 4)
    pygame.draw.circle(screen, ORANGE, (px, py), 12)

    # Flèche de commande totale (u + fu)
    total_force = u + fu
    if abs(total_force) > 0.1:
        arrow_len = int(min(abs(total_force) / 3, 80))
        direction = int(np.sign(total_force))
        ax_end = cx + direction * arrow_len
        color_arrow = RED if fu != 0 else CYAN
        
        pygame.draw.line(screen, color_arrow, (cx, cy - 5), (ax_end, cy - 5), 2)
        pygame.draw.polygon(screen, color_arrow, [
            (ax_end, cy - 5),
            (ax_end - direction*10, cy - 12),
            (ax_end - direction*10, cy + 2)
        ])

    # HUD texte
    deg = np.degrees(th) % 360
    if deg > 180: deg -= 360 # Affichage plus lisible (-180 à 180)

    lines = [
        f"LQR ACTIVÉ",
        f"x      = {x:+.3f} m",
        f"theta  = {deg:+.1f} deg",
        f"x_dot  = {state_point[0]:+.3f} m/s",
        f"th_dot = {state_point[1]:+.3f} rad/s",
        f"u LQR  = {u:+.1f} N",
        f"u Perturb= {fu:+.1f} N",
        "",
        "← → : appliquer une perturbation",
        "R    : réinitialiser",
        "C    : coup violent"
    ]
    for i, line in enumerate(lines):
        color = GREEN if i == 0 else WHITE
        surf = font.render(line, True, color)
        screen.blit(surf, (15, 15 + i * 22))

    pygame.display.flip()


# -----------------------------------------------------------------------
# Boucle principale
# -----------------------------------------------------------------------

running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_r:   # Réinitialiser (légèrement décalé)
                state       = np.array([0.0, 0.1])
                state_point = np.array([0.0, 0.0])
            if event.key == pygame.K_c:   # Coup violent
                state_point[1] += 2.0

    # Lecture continue pour la force de perturbation
    keys = pygame.key.get_pressed()
    if keys[pygame.K_LEFT]:
        fu = -50.0
    elif keys[pygame.K_RIGHT]:
        fu =  50.0
    else:
        fu =  0.0

    state, state_point = runge_kutta_step(state, state_point, fu)
    
    # Recalcule u courant juste pour l'affichage HUD
    u_actuel = Calc_u(state, state_point)
    Afficher(state, state_point, u_actuel, fu)
    
    clock.tick(freq)

pygame.quit()