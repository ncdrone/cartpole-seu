#!/usr/bin/env python3
"""Offline discrete LQR gains for the FSW cart-pole (scipy + numpy required).

Linearizes the SAME frictionless Florian/Barto-Sutton dynamics used in
src/Plant.cpp about upright (finite differences), discretizes with ZOH at DT,
solves the DARE, and prints K for u = -K [x, xdot, theta, thetadot].
"""
import numpy as np
from scipy.linalg import solve_discrete_are, expm

M, m, L, G, DT = 1.0, 0.1, 0.5, 9.81, 0.01
Q = np.diag([1.0, 1.0, 10.0, 1.0])
R = np.array([[0.1]])

def f(s, u):
    _, xd, th, thd = s
    c, sn = np.cos(th), np.sin(th)
    tmp = (u + m * L * thd**2 * sn) / (M + m)
    thdd = (G * sn - c * tmp) / (L * (4.0 / 3.0 - m * c**2 / (M + m)))
    xdd = tmp - m * L * thdd * c / (M + m)
    return np.array([xd, xdd, thd, thdd])

e = 1e-6
A = np.column_stack([(f(np.eye(4)[i]*e, 0) - f(-np.eye(4)[i]*e, 0)) / (2*e) for i in range(4)])
B = ((f(np.zeros(4), e) - f(np.zeros(4), -e)) / (2*e)).reshape(4, 1)

M_ = expm(np.block([[A, B], [np.zeros((1, 5))]]) * DT)
Ad, Bd = M_[:4, :4], M_[:4, 4:]
P = solve_discrete_are(Ad, Bd, Q, R)
K = np.linalg.solve(R + Bd.T @ P @ Bd, Bd.T @ P @ Ad)
print("Q =", np.diag(Q), "R =", R[0, 0])
print("K = [%.6f, %.6f, %.6f, %.6f]" % tuple(K[0]))
print("|eig(Ad-Bd K)| =", np.abs(np.linalg.eigvals(Ad - Bd @ K)))
