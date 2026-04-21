# -*- coding: utf-8 -*-
"""
Simulation d'un pendule simple inversé : Swing-Up + LQR + Looping
=================================================================
Pendule simple sur chariot avec résolution RK4.
Le système part d'en bas, fait un swing-up, et se stabilise (LQR).
Appuyez sur 'L' pour forcer un looping automatique !

Contrôle : 
- touches ← → : perturbations manuelles
- R : Réinitialiser en bas
- C : Coup violent (vitesse)
- L : Lancer un looping 360°

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

# Frottements
b0, b1   = 0.5, 0.025   # Visqueux
fs0, fs1 = 0.1, 0.0125  # Secs

dt   = 0.01        # Pas de temps (s)
freq = int(1 / dt) # Fréquence d'affichage (Hz)

# --- État initial : pendule en bas ---
state       = np.array([0.0, np.pi + 0.01])   # [x, theta] 
state_point = np.array([0.0, 0.0])            # [x_dot, theta_dot]
state_eq    = np.array([0.0, 0.0])            # Équilibre visé (haut)

# --- Variable globale pour le looping ---
loop_phase = 0  # 0: inactif, 1: kick de départ, 2: accompagnement

# -----------------------------------------------------------------------
# LQR : Calcul du gain K par linéarisation autour du point haut (theta=0)
# -----------------------------------------------------------------------

Q = np.diag([10, 100, 1, 1])
R = np.array([[0.01]])

def lqr(A, B, Q, R):
    S = solve_continuous_are(A, B, Q, R)
    K = (1/R) * (B.T @ S)
    return K

def Calc_K(Q, R):
    Deq = np.array([[m0 + m1, 0.5 * m1 * L1],
                    [0.5 * m1 * L1, (1/3) * m1 * L1**2]])
    D_inv = np.linalg.inv(Deq)
    M_pos = np.array([[0, 0], [0, 0.5 * m1 * L1 * g]])
    M_vel = np.array([[-b0, 0], [0, -b1]])
    H_vec = np.array([[1], [0]])

    A21 = D_inv @ M_pos
    A22 = D_inv @ M_vel
    B2  = D_inv @ H_vec

    A = np.block([[np.zeros((2, 2)), np.eye(2)], [A21, A22]])
    B = np.vstack((np.zeros((2, 1)), B2))
    return lqr(A, B, Q, R)

K = Calc_K(Q, R)

# -----------------------------------------------------------------------
# Physique Non-Linéaire
# -----------------------------------------------------------------------

def Calc_D(state):
    th = state[1]
    return np.array([[m0 + m1, (0.5*m1)*L1*np.cos(th)],
                     [(0.5*m1)*L1*np.cos(th), (1/3)*m1*L1**2]])

def Calc_C(state, state_point):
    return np.array([[0, -(0.5*m1)*L1*np.sin(state[1])*state_point[1]],
                     [0, 0]])

def Calc_G(state):
    return np.array([0, -0.5 * m1 * L1 * g * np.sin(state[1])])

H_matrix = np.array([1.0, 0.0])

def calc_friction(state_point):
    return np.array([-b0 * state_point[0] - fs0 * np.sign(state_point[0]),
                     -b1 * state_point[1] - fs1 * np.sign(state_point[1])])

def Calc_accelerations(state, state_point, u, fu):
    D_inv = np.linalg.inv(Calc_D(state))
    acc = (- D_inv @ (Calc_C(state, state_point) @ state_point)
           - D_inv @ Calc_G(state)
           + D_inv @ (H_matrix * (u + fu))
           + D_inv @ calc_friction(state_point))
    return acc

# -----------------------------------------------------------------------
# Contrôleurs : LQR, Swing-Up et Looping
# -----------------------------------------------------------------------

def Calc_u_LQR(state, state_point):
    err_th = (state[1] - state_eq[1] + np.pi) % (2 * np.pi) - np.pi
    X_err = np.array([state[0] - state_eq[0], err_th, state_point[0], state_point[1]])
    return -(K @ X_err)[0]

def Calc_u_SwingUp(state, state_point):
    x, th = state[0], state[1]
    xd, thd = state_point[0], state_point[1]
    
    Ip = (1/3) * m1 * L1**2
    E = 0.5 * Ip * thd**2 + m1 * g * l1 * (np.cos(th) - 1)
    
    u_pump = 50.0 * E * thd * np.cos(th)
    u_pump = np.clip(u_pump, -120, 120) 
    
    u_cart = -15.0 * x - 10.0 * xd
    u_cart = np.clip(u_cart, -50, 50)
    
    return u_pump + u_cart

def Get_Control_Mode(state, state_point):
    global loop_phase
    th_norm = (state[1] + np.pi) % (2 * np.pi) - np.pi
    
    # 1. Gestion du LOOPING Automatique
    if loop_phase > 0:
        if loop_phase == 1:
            # Phase 1: Kick massif pour expulser le pendule du LQR
            u_loop = 250.0 
            # Dès qu'il penche assez ou prend de la vitesse, on passe en accompagnement
            if abs(th_norm) > 0.5 or abs(state_point[1]) > 2.0:
                loop_phase = 2
            return "LOOPING", u_loop
            
        elif loop_phase == 2:
            # Phase 2: Accompagner la chute pour faire le tour complet
            u_pump = 80.0 * state_point[1] * np.cos(state[1])
            u_pump = np.clip(u_pump, -150, 150)
            
            # Limiter la fuite du chariot pendant le looping
            u_cart = -15.0 * state[0] - 10.0 * state_point[0]
            u_loop = u_pump + np.clip(u_cart, -50, 50)
            
            # Phase 3: S'il a fini son tour et remonte vers le top, on relâche
            if abs(th_norm) < 0.3 and abs(state_point[1]) < 6.0:
                loop_phase = 0 # Fin du looping, on laisse l'algo naturel attraper
            return "LOOPING", u_loop

    # 2. Comportement par défaut (Capture LQR ou Swing-Up)
    if abs(th_norm) < 0.3 and abs(state_point[1]) < 4.0:
        return "LQR", Calc_u_LQR(state, state_point)
    else:
        return "SWING-UP", Calc_u_SwingUp(state, state_point)

# -----------------------------------------------------------------------
# Intégrateur Runge-Kutta 4
# -----------------------------------------------------------------------

def runge_kutta_step(state, state_point, fu):
    _, u1 = Get_Control_Mode(state, state_point)
    k1s = state_point
    k1d = Calc_accelerations(state, state_point, u1, fu)

    s2  = state + 0.5*dt*k1s
    sd2 = state_point + 0.5*dt*k1d
    _, u2 = Get_Control_Mode(s2, sd2)
    k2s = sd2
    k2d = Calc_accelerations(s2, sd2, u2, fu)

    s3  = state + 0.5*dt*k2s
    sd3 = state_point + 0.5*dt*k2d
    _, u3 = Get_Control_Mode(s3, sd3)
    k3s = sd3
    k3d = Calc_accelerations(s3, sd3, u3, fu)

    s4  = state + dt*k3s
    sd4 = state_point + dt*k3d
    _, u4 = Get_Control_Mode(s4, sd4)
    k4s = sd4
    k4d = Calc_accelerations(s4, sd4, u4, fu)

    new_state       = state + (dt/6)*(k1s + 2*k2s + 2*k3s + k4s)
    new_state_point = state_point + (dt/6)*(k1d + 2*k2d + 2*k3d + k4d)
    return new_state, new_state_point

# -----------------------------------------------------------------------
# Affichage Pygame
# -----------------------------------------------------------------------

pygame.init()
WIDTH, HEIGHT = 1400, 700
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Pendule Simple : Swing-Up + LQR + Looping")
clock = pygame.time.Clock()
font  = pygame.font.SysFont("monospace", 18)

BG, WHITE, ORANGE, GREEN, GRAY = (50,50,50), (255,255,255), (255,165,0), (125,125,0), (150,150,150)
CYAN, RED, PURPLE, YELLOW = (0,200,220), (255,50,50), (180,50,255), (255,220,0)

pivot_x, pivot_y = WIDTH // 2, HEIGHT // 2
chariot_w, chariot_h, wheel_r = 60, 24, 9
pend_px = L1 * 100

def Afficher(state, state_point, mode, u, fu):
    x, th = state[0], state[1]
    cx = int(pivot_x + x * 100)
    px = int(cx + pend_px * np.sin(th))
    py = int(pivot_y - pend_px * np.cos(th))

    screen.fill(BG)

    # Décor et Chariot
    pygame.draw.line(screen, GRAY, (0, pivot_y + chariot_h//2 + wheel_r), (WIDTH, pivot_y + chariot_h//2 + wheel_r), 4)
    pygame.draw.rect(screen, GREEN, (cx - chariot_w//2, pivot_y - chariot_h//2, chariot_w, chariot_h))
    for wx in [cx - chariot_w//3, cx + chariot_w//3]:
        pygame.draw.circle(screen, WHITE, (wx, pivot_y + chariot_h//2 + wheel_r), wheel_r)
    
    # Pendule
    pygame.draw.line(screen, WHITE, (cx, pivot_y), (px, py), 4)
    pygame.draw.circle(screen, ORANGE, (px, py), 12)

    # Flèche de commande totale
    total_force = u + fu
    if abs(total_force) > 0.1:
        arrow_len = int(min(abs(total_force) / 3, 80))
        direction = int(np.sign(total_force))
        ax_end = cx + direction * arrow_len
        
        # Choix de la couleur selon le mode actif
        if fu != 0: color_arrow = RED
        elif mode == "LQR": color_arrow = CYAN
        elif mode == "LOOPING": color_arrow = YELLOW
        else: color_arrow = PURPLE
        
        pygame.draw.line(screen, color_arrow, (cx, pivot_y - 5), (ax_end, pivot_y - 5), 2)
        pygame.draw.polygon(screen, color_arrow, [(ax_end, pivot_y - 5), (ax_end - direction*10, pivot_y - 12), (ax_end - direction*10, pivot_y + 2)])

    # HUD
    deg = np.degrees(th) % 360
    if deg > 180: deg -= 360
    lines = [
        f"MODE ACTIF : {mode}",
        f"x      = {x:+.3f} m",
        f"theta  = {deg:+.1f} deg",
        f"u auto = {u:+.1f} N",
        f"u user = {fu:+.1f} N",
        "",
        "← → : Perturbation",
        "L   : Lancer un LOOPING ! 🚀",
        "R   : Remettre en bas",
        "C   : Coup violent"
    ]
    for i, line in enumerate(lines):
        color = WHITE
        if i == 0:
            if mode == "LQR": color = CYAN
            elif mode == "LOOPING": color = YELLOW
            else: color = PURPLE
        elif i == 7: color = YELLOW # Mise en valeur de la touche L
        
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
            if event.key == pygame.K_r:   
                state, state_point = np.array([0.0, np.pi + 0.01]), np.array([0.0, 0.0])
                loop_phase = 0
            if event.key == pygame.K_c:   
                state_point[1] += 4.0
            if event.key == pygame.K_l:   # --- DÉCLENCHEMENT DU LOOPING ---
                loop_phase = 1

    keys = pygame.key.get_pressed()
    fu = -80.0 if keys[pygame.K_LEFT] else (80.0 if keys[pygame.K_RIGHT] else 0.0)

    # Intégration RK4
    state, state_point = runge_kutta_step(state, state_point, fu)
    
    current_mode, u_actuel = Get_Control_Mode(state, state_point)
    Afficher(state, state_point, current_mode, u_actuel, fu)
    
    clock.tick(freq)

pygame.quit()