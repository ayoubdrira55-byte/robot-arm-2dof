"""Intégration temporelle (Runge-Kutta 4) de la boucle fermée robot + correcteur."""
import numpy as np


def simulate(robot, controller, reference, q0, dq0=(0, 0), T=6.0, dt=1e-3, disturbance=None):
    """Simule la boucle fermée.

    Le correcteur est échantillonné à dt (commande numérique, couple bloqué
    sur la période), le robot est intégré par RK4.
    disturbance(t) -> couple de perturbation externe (optionnel).
    """
    n = int(T / dt) + 1
    t = np.linspace(0, T, n)
    q = np.zeros((n, 2)); dq = np.zeros((n, 2))
    q_ref = np.zeros((n, 2)); tau = np.zeros((n, 2))
    q[0], dq[0] = q0, dq0
    controller.reset()

    def f(state, u):
        qq, vv = state[:2], state[2:]
        return np.concatenate([vv, robot.forward_dynamics(qq, vv, u)])

    for k in range(n - 1):
        qr, dqr, ddqr = reference(t[k])
        q_ref[k] = qr
        u = controller(t[k], dt, q[k], dq[k], qr, dqr, ddqr)
        tau[k] = u
        u_total = u + (disturbance(t[k]) if disturbance else 0)

        s = np.concatenate([q[k], dq[k]])
        k1 = f(s, u_total)
        k2 = f(s + dt / 2 * k1, u_total)
        k3 = f(s + dt / 2 * k2, u_total)
        k4 = f(s + dt * k3, u_total)
        s = s + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
        q[k + 1], dq[k + 1] = s[:2], s[2:]

    q_ref[-1] = reference(t[-1])[0]
    tau[-1] = tau[-2]
    return {"t": t, "q": q, "dq": dq, "q_ref": q_ref, "tau": tau}


def metrics(res, settle_band=0.02):
    """Indicateurs de performance par articulation."""
    e = res["q_ref"] - res["q"]
    t = res["t"]
    out = {
        "erreur_RMS_deg": np.degrees(np.sqrt(np.mean(e**2, axis=0))),
        "erreur_max_deg": np.degrees(np.max(np.abs(e), axis=0)),
        "energie_J": float(np.sum(np.abs(res["tau"] * res["dq"])) * (t[1] - t[0])),
    }
    return out
