# -*- coding: utf-8 -*-
"""
Simulation d'un pendule simple inversé sur un chariot
======================================================
Pendule simple sur chariot avec résolution RK4.
Contrôle : touches ← → pour appliquer une force au chariot.

Modules requis : numpy, pygame
"""

import numpy as np
import pygame

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

# --- État initial : pendule en position haute (π = 180°) ---
state       = np.array([0.0, np.pi])   # [x, theta]
state_point = np.array([0.0, 0.0])     # [x_dot, theta_dot]
u = 0.0


# -----------------------------------------------------------------------
# Physique : matrices D, C, G  (modèle Lagrangien du pendule sur chariot)
# -----------------------------------------------------------------------

def Calc_D(state):
    """Matrice d'inertie 2×2."""
    th = state[1]
    D = np.array([
        [m0 + m1,                  (0.5*m1)*L1*np.cos(th)],
        [(0.5*m1)*L1*np.cos(th),   (1/3)*m1*L1**2        ]
    ])
    return D

def Calc_C(state, state_point):
    """Matrice de Coriolis 2×2."""
    th  = state[1]
    thd = state_point[1]
    C = np.array([
        [0, -(0.5*m1)*L1*np.sin(th)*thd],
        [0,  0                          ]
    ])
    return C

def Calc_G(state):
    """Vecteur gravitationnel 2×1."""
    th = state[1]
    G = np.array([
        0,
        -0.5 * m1 * L1 * g * np.sin(th)
    ])
    return G

H = np.array([1.0, 0.0])  # Force u appliquée uniquement sur le chariot

def calc_friction(state_point):
    """Frottements visqueux + secs."""
    return np.array([
        -b0 * state_point[0] - fs0 * np.sign(state_point[0]),
        -b1 * state_point[1] - fs1 * np.sign(state_point[1])
    ])

def Calc_accelerations(state, state_point, u):
    D     = Calc_D(state)
    C     = Calc_C(state, state_point)
    G     = Calc_G(state)
    D_inv = np.linalg.inv(D)
    f     = calc_friction(state_point)

    acc = (- D_inv @ (C @ state_point)
           - D_inv @ G
           + D_inv @ (H * u)
           + D_inv @ f)
    return acc


# -----------------------------------------------------------------------
# Intégrateur Runge-Kutta 4
# -----------------------------------------------------------------------

def runge_kutta_step(state, state_point, u):
    k1s = state_point
    k1d = Calc_accelerations(state, state_point, u)

    s2  = state       + 0.5*dt*k1s
    sd2 = state_point + 0.5*dt*k1d
    k2s = sd2
    k2d = Calc_accelerations(s2, sd2, u)

    s3  = state       + 0.5*dt*k2s
    sd3 = state_point + 0.5*dt*k2d
    k3s = sd3
    k3d = Calc_accelerations(s3, sd3, u)

    s4  = state       + dt*k3s
    sd4 = state_point + dt*k3d
    k4s = sd4
    k4d = Calc_accelerations(s4, sd4, u)

    new_state       = state       + (dt/6)*(k1s + 2*k2s + 2*k3s + k4s)
    new_state_point = state_point + (dt/6)*(k1d + 2*k2d + 2*k3d + k4d)
    return new_state, new_state_point


# -----------------------------------------------------------------------
# Affichage Pygame
# -----------------------------------------------------------------------

def Conversion_MP(val):
    """Mètres → pixels (1 m = 100 px)."""
    return val * 100

pygame.init()
WIDTH, HEIGHT = 1400, 700
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Pendule Simple sur Chariot")
clock = pygame.time.Clock()
font  = pygame.font.SysFont("monospace", 18)

# Couleurs
BG     = ( 50,  50,  50)
WHITE  = (255, 255, 255)
ORANGE = (255, 165,   0)
GREEN  = (125, 125,   0)
GRAY   = (150, 150, 150)
CYAN   = (  0, 200, 220)

pivot_x = WIDTH  // 2
pivot_y = HEIGHT // 2
chariot_w, chariot_h = 60, 24
wheel_r = 9
pend_px = Conversion_MP(L1)


def Afficher(state, state_point, u):
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

    # Flèche de force
    if abs(u) > 0.1:
        arrow_len = int(min(abs(u) / 3, 80))
        direction = int(np.sign(u))
        ax_end = cx + direction * arrow_len
        pygame.draw.line(screen, CYAN, (cx, cy - 5), (ax_end, cy - 5), 2)
        pygame.draw.polygon(screen, CYAN, [
            (ax_end, cy - 5),
            (ax_end - direction*10, cy - 12),
            (ax_end - direction*10, cy + 2)
        ])

    # HUD texte
    deg = np.degrees(th) % 360
    lines = [
        f"x      = {x:+.3f} m",
        f"theta  = {deg:.1f} deg",
        f"x_dot  = {state_point[0]:+.3f} m/s",
        f"th_dot = {state_point[1]:+.3f} rad/s",
        f"u      = {u:+.1f} N",
        "",
        "← → : appliquer une force",
        "R    : réinitialiser",
    ]
    for i, line in enumerate(lines):
        surf = font.render(line, True, WHITE)
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
            if event.key == pygame.K_r:   # Réinitialiser
                state       = np.array([0.0, np.pi])
                state_point = np.array([0.0, 0.0])

    keys = pygame.key.get_pressed()
    if keys[pygame.K_LEFT]:
        u = -200.0
    elif keys[pygame.K_RIGHT]:
        u =  200.0
    else:
        u =  0.0

    state, state_point = runge_kutta_step(state, state_point, u)
    Afficher(state, state_point, u)
    clock.tick(freq)

pygame.quit()
