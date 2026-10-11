"""Génération de trajectoires de référence."""
import numpy as np

from robot import TwoLinkArm


def quintic(q0, qf, T):
    """Polynôme de degré 5 : vitesse et accélération nulles au départ et à l'arrivée."""
    q0, qf = np.asarray(q0, float), np.asarray(qf, float)

    def ref(t):
        s = np.clip(t / T, 0, 1)
        d = qf - q0
        pos = q0 + d * (10 * s**3 - 15 * s**4 + 6 * s**5)
        vel = d * (30 * s**2 - 60 * s**3 + 30 * s**4) / T if t < T else 0 * d
        acc = d * (60 * s - 180 * s**2 + 120 * s**3) / T**2 if t < T else 0 * d
        return pos, vel, acc

    return ref


def cartesian_circle(robot: TwoLinkArm, center=(1.0, 0.4), radius=0.35, period=4.0):
    """Cercle dans l'espace cartésien converti en consignes articulaires.

    Vitesses et accélérations articulaires obtenues par le jacobien :
    q' = J⁻¹ x'   et   q'' = J⁻¹ (x'' - J' q')
    """
    cx, cy = center
    w = 2 * np.pi / period
    eps = 1e-5

    def ref(t):
        x = np.array([cx + radius * np.cos(w * t), cy + radius * np.sin(w * t)])
        dx = np.array([-radius * w * np.sin(w * t), radius * w * np.cos(w * t)])
        ddx = np.array([-radius * w**2 * np.cos(w * t), -radius * w**2 * np.sin(w * t)])
        q = robot.inverse_kinematics(*x)
        J = robot.jacobian(q)
        dq = np.linalg.solve(J, dx)
        dJ = (robot.jacobian(q + eps * dq) - J) / eps  # dérivée numérique de J
        ddq = np.linalg.solve(J, ddx - dJ @ dq)
        return q, dq, ddq

    ref.cartesian = lambda t: (cx + radius * np.cos(w * t), cy + radius * np.sin(w * t))
    return ref
