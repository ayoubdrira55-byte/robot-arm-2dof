"""Lance toutes les expériences et génère les figures dans results/."""
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation, PillowWriter

from controllers import PID, ComputedTorque, PDGravity
from robot import RobotParams, TwoLinkArm
from simulator import metrics, simulate
from trajectory import cartesian_circle, quintic

OUT = Path(__file__).resolve().parent.parent / "results"
OUT.mkdir(exist_ok=True)
COLORS = {"PID": "#2a6fdb", "PD + compensation gravité": "#e8890c", "Couple calculé": "#1b9e5a"}
plt.rcParams.update({"figure.dpi": 110, "axes.grid": True, "grid.alpha": 0.3, "font.size": 10})


def controllers_for(robot):
    return [PID(robot), PDGravity(robot), ComputedTorque(robot)]


# ---------------------------------------------------------------------- #
# 1. Validation du modèle : conservation de l'énergie sans frottement
# ---------------------------------------------------------------------- #
def validate_model():
    robot = TwoLinkArm(RobotParams(b=0.0))

    class Zero:
        def reset(self): pass
        def __call__(self, *a): return np.zeros(2)

    res = simulate(robot, Zero(), lambda t: (np.zeros(2),) * 3, q0=(0.3, 0.5), T=5)
    E = np.array([robot.energy(q, dq) for q, dq in zip(res["q"], res["dq"])])
    drift = np.max(np.abs(E - E[0])) / abs(E[0]) * 100

    fig, ax = plt.subplots(1, 2, figsize=(11, 3.6))
    ax[0].plot(res["t"], np.degrees(res["q"][:, 0]), label="q1")
    ax[0].plot(res["t"], np.degrees(res["q"][:, 1]), label="q2")
    ax[0].set(title="Chute libre (couple nul, sans frottement)", xlabel="t [s]", ylabel="angle [°]")
    ax[0].legend()
    ax[1].plot(res["t"], E, color="#c0392b")
    ax[1].set(title=f"Énergie totale — dérive max {drift:.1e} %", xlabel="t [s]", ylabel="E [J]")
    ax[1].set_ylim(E[0] - 1, E[0] + 1)
    fig.tight_layout(); fig.savefig(OUT / "01_validation_energie.png"); plt.close(fig)
    return drift


# ---------------------------------------------------------------------- #
# 2. Mouvement point à point + perturbation
# ---------------------------------------------------------------------- #
def point_to_point():
    robot = TwoLinkArm()
    q0, qf = np.radians([-90, 0]), np.radians([45, 60])
    ref = quintic(q0, qf, T=2.0)
    dist = lambda t: np.array([0, 15.0]) if 3.5 <= t < 3.8 else np.zeros(2)  # choc sur l'axe 2

    results = {}
    fig, ax = plt.subplots(2, 2, figsize=(12, 7), sharex=True)
    for c in controllers_for(robot):
        r = simulate(robot, c, ref, q0=q0, T=6, disturbance=dist)
        results[c.name] = r
        col = COLORS[c.name]
        for j in range(2):
            ax[0, j].plot(r["t"], np.degrees(r["q"][:, j]), color=col, label=c.name)
            ax[1, j].plot(r["t"], r["tau"][:, j], color=col)
    for j in range(2):
        ax[0, j].plot(r["t"], np.degrees(r["q_ref"][:, j]), "k--", lw=1, label="consigne")
        ax[0, j].set(title=f"Articulation {j+1}", ylabel="angle [°]")
        ax[1, j].set(xlabel="t [s]", ylabel="couple [N.m]")
        for a in ax[:, j]:
            a.axvspan(3.5, 3.8, color="red", alpha=0.08)
    ax[0, 0].legend(fontsize=8)
    fig.suptitle("Mouvement point à point (polynôme quintique) + perturbation à t = 3,5 s", fontweight="bold")
    fig.tight_layout(); fig.savefig(OUT / "02_point_a_point.png"); plt.close(fig)
    return results


# ---------------------------------------------------------------------- #
# 3. Suivi de trajectoire circulaire dans l'espace cartésien
# ---------------------------------------------------------------------- #
def circle_tracking():
    robot = TwoLinkArm()
    ref = cartesian_circle(robot)
    q0 = ref(0)[0]
    results = {}

    fig, ax = plt.subplots(1, 2, figsize=(12, 4.8))
    tt = np.linspace(0, 4, 300)
    ax[0].plot(*np.array([ref.cartesian(t) for t in tt]).T, "k--", lw=1.2, label="consigne")
    for c in controllers_for(robot):
        r = simulate(robot, c, ref, q0=q0, T=8)
        results[c.name] = r
        xy = np.array([robot.forward_kinematics(q)[1] for q in r["q"]])
        xy_ref = np.array([ref.cartesian(t) for t in r["t"]])
        err_mm = np.linalg.norm(xy - xy_ref, axis=1) * 1000
        r["err_mm"] = err_mm
        ax[0].plot(*xy.T, color=COLORS[c.name], lw=1.5, label=c.name)
        ax[1].semilogy(r["t"], err_mm + 1e-6, color=COLORS[c.name], label=c.name)
    ax[0].set(title="Trajectoire de l'effecteur", xlabel="x [m]", ylabel="y [m]", aspect="equal")
    ax[0].legend(fontsize=8)
    ax[1].set(title="Erreur de position de l'effecteur", xlabel="t [s]", ylabel="erreur [mm] (échelle log)")
    fig.tight_layout(); fig.savefig(OUT / "03_suivi_cercle.png"); plt.close(fig)
    return robot, ref, results


# ---------------------------------------------------------------------- #
# 4. Animation
# ---------------------------------------------------------------------- #
def animate(robot, ref, res, name):
    q, t = res["q"], res["t"]
    idx = np.arange(0, len(t), 40)  # 25 images/s
    L = robot.p.l1 + robot.p.l2
    fig, ax = plt.subplots(figsize=(5, 5))
    ax.set(xlim=(-0.4, L + 0.1), ylim=(-0.6, L), aspect="equal", title=f"Bras 2 axes — {name}")
    tt = np.linspace(0, 4, 200)
    ax.plot(*np.array([ref.cartesian(s) for s in tt]).T, "k--", lw=1, alpha=0.5)
    arm, = ax.plot([], [], "-o", lw=5, color="#34495e", mfc="#e74c3c", ms=9)
    trace, = ax.plot([], [], color=COLORS[name], lw=1.5)
    txt = ax.text(0.02, 0.95, "", transform=ax.transAxes)
    path = []

    def update(i):
        k = idx[i]
        elbow, eff = robot.forward_kinematics(q[k])
        arm.set_data([0, elbow[0], eff[0]], [0, elbow[1], eff[1]])
        path.append(eff)
        trace.set_data(*np.array(path).T)
        txt.set_text(f"t = {t[k]:.2f} s")
        return arm, trace, txt

    anim = FuncAnimation(fig, update, frames=len(idx), blit=True)
    anim.save(OUT / "04_animation.gif", writer=PillowWriter(fps=25))
    plt.close(fig)


def write_metrics(drift, p2p, circ):
    lines = ["# Résultats", "", f"Validation du modèle : dérive d'énergie max = {drift:.1e} %", "",
             "## Mouvement point à point (avec perturbation)", "",
             "| Correcteur | Erreur RMS q1 [°] | Erreur RMS q2 [°] | Erreur max q2 [°] | Énergie [J] |",
             "|---|---|---|---|---|"]
    for name, r in p2p.items():
        m = metrics(r)
        lines.append(f"| {name} | {m['erreur_RMS_deg'][0]:.3f} | {m['erreur_RMS_deg'][1]:.3f} | "
                     f"{m['erreur_max_deg'][1]:.3f} | {m['energie_J']:.1f} |")
    lines += ["", "## Suivi de cercle (régime établi, t > 4 s)", "",
              "| Correcteur | Erreur moyenne [mm] | Erreur max [mm] |", "|---|---|---|"]
    for name, r in circ.items():
        e = r["err_mm"][r["t"] > 4]
        lines.append(f"| {name} | {e.mean():.2f} | {e.max():.2f} |")
    text = "\n".join(lines) + "\n"
    (OUT / "metrics.md").write_text(text, encoding="utf-8")
    print(text)


if __name__ == "__main__":
    drift = validate_model()
    p2p = point_to_point()
    robot, ref, circ = circle_tracking()
    animate(robot, ref, circ["Couple calculé"], "Couple calculé")
    write_metrics(drift, p2p, circ)
    print(f"Figures enregistrées dans {OUT}")
