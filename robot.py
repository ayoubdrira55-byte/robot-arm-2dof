"""Modèle d'un bras robotique planaire à 2 degrés de liberté (RR) dans un plan vertical.

Contient :
- la cinématique directe et inverse,
- le jacobien,
- le modèle dynamique de Lagrange : M(q) q'' + C(q, q') q' + G(q) = tau
"""
from dataclasses import dataclass

import numpy as np


@dataclass
class RobotParams:
    l1: float = 1.0      # longueur segment 1 [m]
    l2: float = 0.8      # longueur segment 2 [m]
    m1: float = 2.0      # masse segment 1 [kg]
    m2: float = 1.5      # masse segment 2 [kg]
    g: float = 9.81      # gravité [m/s²]
    b: float = 0.5       # frottement visqueux articulaire [N.m.s/rad]

    @property
    def lc1(self):  # centre de masse (segment homogène)
        return self.l1 / 2

    @property
    def lc2(self):
        return self.l2 / 2

    @property
    def I1(self):   # inertie d'une tige autour de son centre
        return self.m1 * self.l1 ** 2 / 12

    @property
    def I2(self):
        return self.m2 * self.l2 ** 2 / 12


class TwoLinkArm:
    def __init__(self, params: RobotParams | None = None):
        self.p = params or RobotParams()

    # ------------------------------------------------------------------ #
    # Cinématique
    # ------------------------------------------------------------------ #
    def forward_kinematics(self, q):
        """Retourne les positions du coude et de l'effecteur."""
        q1, q2 = q
        l1, l2 = self.p.l1, self.p.l2
        elbow = np.array([l1 * np.cos(q1), l1 * np.sin(q1)])
        effector = elbow + np.array([l2 * np.cos(q1 + q2), l2 * np.sin(q1 + q2)])
        return elbow, effector

    def inverse_kinematics(self, x, y, elbow_up=True):
        """Résout q1, q2 pour atteindre (x, y). Lève ValueError hors espace de travail."""
        l1, l2 = self.p.l1, self.p.l2
        c2 = (x ** 2 + y ** 2 - l1 ** 2 - l2 ** 2) / (2 * l1 * l2)
        if abs(c2) > 1:
            raise ValueError(f"Point ({x:.2f}, {y:.2f}) hors de l'espace de travail")
        s2 = np.sqrt(1 - c2 ** 2) * (1 if elbow_up else -1)
        q2 = np.arctan2(s2, c2)
        q1 = np.arctan2(y, x) - np.arctan2(l2 * s2, l1 + l2 * c2)
        return np.array([q1, q2])

    def jacobian(self, q):
        q1, q2 = q
        l1, l2 = self.p.l1, self.p.l2
        s1, c1 = np.sin(q1), np.cos(q1)
        s12, c12 = np.sin(q1 + q2), np.cos(q1 + q2)
        return np.array([
            [-l1 * s1 - l2 * s12, -l2 * s12],
            [l1 * c1 + l2 * c12, l2 * c12],
        ])

    # ------------------------------------------------------------------ #
    # Dynamique
    # ------------------------------------------------------------------ #
    def mass_matrix(self, q):
        p = self.p
        c2 = np.cos(q[1])
        m11 = p.m1 * p.lc1 ** 2 + p.I1 + p.m2 * (p.l1 ** 2 + p.lc2 ** 2 + 2 * p.l1 * p.lc2 * c2) + p.I2
        m12 = p.m2 * (p.lc2 ** 2 + p.l1 * p.lc2 * c2) + p.I2
        m22 = p.m2 * p.lc2 ** 2 + p.I2
        return np.array([[m11, m12], [m12, m22]])

    def coriolis(self, q, dq):
        """Vecteur C(q, q') q' (Coriolis + centrifuge)."""
        p = self.p
        h = -p.m2 * p.l1 * p.lc2 * np.sin(q[1])
        return np.array([
            h * dq[1] * (2 * dq[0] + dq[1]),
            -h * dq[0] ** 2,
        ])

    def gravity(self, q):
        p = self.p
        c1, c12 = np.cos(q[0]), np.cos(q[0] + q[1])
        g2 = p.m2 * p.lc2 * p.g * c12
        g1 = (p.m1 * p.lc1 + p.m2 * p.l1) * p.g * c1 + g2
        return np.array([g1, g2])

    def friction(self, dq):
        return self.p.b * np.asarray(dq)

    def forward_dynamics(self, q, dq, tau):
        """q'' = M⁻¹ (tau - C q' - G - F)."""
        rhs = tau - self.coriolis(q, dq) - self.gravity(q) - self.friction(dq)
        return np.linalg.solve(self.mass_matrix(q), rhs)

    def energy(self, q, dq):
        """Énergie mécanique totale (utile pour valider le modèle)."""
        p = self.p
        kinetic = 0.5 * dq @ self.mass_matrix(q) @ dq
        y1 = p.lc1 * np.sin(q[0])
        y2 = p.l1 * np.sin(q[0]) + p.lc2 * np.sin(q[0] + q[1])
        potential = p.m1 * p.g * y1 + p.m2 * p.g * y2
        return kinetic + potential
