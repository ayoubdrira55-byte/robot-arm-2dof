"""Lois de commande articulaires pour le bras 2 axes."""
import numpy as np

from robot import TwoLinkArm


class Controller:
    name = "base"

    def __init__(self, robot: TwoLinkArm, tau_max: float = 60.0):
        self.robot = robot
        self.tau_max = tau_max  # saturation des actionneurs [N.m]

    def reset(self):
        pass

    def saturate(self, tau):
        return np.clip(tau, -self.tau_max, self.tau_max)

    def __call__(self, t, dt, q, dq, q_ref, dq_ref, ddq_ref):
        raise NotImplementedError


class PID(Controller):
    """PID classique par articulation, sans connaissance du modèle."""
    name = "PID"

    def __init__(self, robot, kp=(150, 80), ki=(60, 30), kd=(25, 12), **kw):
        super().__init__(robot, **kw)
        self.kp, self.ki, self.kd = map(np.array, (kp, ki, kd))
        self.reset()

    def reset(self):
        self.integral = np.zeros(2)

    def __call__(self, t, dt, q, dq, q_ref, dq_ref, ddq_ref):
        e, de = q_ref - q, dq_ref - dq
        self.integral += e * dt
        tau = self.kp * e + self.ki * self.integral + self.kd * de
        return self.saturate(tau)


class PDGravity(Controller):
    """PD + compensation de gravité : stabilité garantie (Lyapunov) en régulation."""
    name = "PD + compensation gravité"

    def __init__(self, robot, kp=(150, 80), kd=(25, 12), **kw):
        super().__init__(robot, **kw)
        self.kp, self.kd = np.array(kp), np.array(kd)

    def __call__(self, t, dt, q, dq, q_ref, dq_ref, ddq_ref):
        e, de = q_ref - q, dq_ref - dq
        tau = self.kp * e + self.kd * de + self.robot.gravity(q)
        return self.saturate(tau)


class ComputedTorque(Controller):
    """Commande par couple calculé (linéarisation par retour d'état).

    tau = M(q) [q''_ref + Kd e' + Kp e] + C(q, q') q' + G(q) + F(q')
    → la dynamique de l'erreur devient linéaire : e'' + Kd e' + Kp e = 0
    Gains choisis pour un pôle double en -wn (amortissement critique).
    """
    name = "Couple calculé"

    def __init__(self, robot, wn=12.0, **kw):
        super().__init__(robot, **kw)
        self.kp = wn ** 2
        self.kd = 2 * wn

    def __call__(self, t, dt, q, dq, q_ref, dq_ref, ddq_ref):
        r = self.robot
        e, de = q_ref - q, dq_ref - dq
        v = ddq_ref + self.kd * de + self.kp * e
        tau = r.mass_matrix(q) @ v + r.coriolis(q, dq) + r.gravity(q) + r.friction(dq)
        return self.saturate(tau)
