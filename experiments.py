"""
Experiments used to answer the questions of Assignment 01 (results in risultati.md, report in report/).

    python3 experiments.py q1      # (q2, q3, q4, q5 are added question by question)

Every case is simulated without viewer, starting from the default parameters of romeo_conf.py
(the module is reloaded before each case, so romeo_conf.py itself is never modified). The script
prints the performance indicators and asks whether to save the figures in figures/.
"""
import importlib
import os
import sys
from contextlib import contextmanager

import matplotlib.pyplot as plt
import numpy as np

import biped_walking
import plot_utils as plut
import romeo_conf
import tray_sinusoid
import tsid_biped

HERE = os.path.dirname(os.path.abspath(__file__))
FIG_DIR = os.path.join(HERE, "figures")


# ------------------------------------------------------------------------------------------------
# helpers
# ------------------------------------------------------------------------------------------------
def make_conf(**overrides):
    '''Default configuration of romeo_conf.py (reloaded), without viewer, plus the given overrides.'''
    conf = importlib.reload(romeo_conf)
    conf.viewer = None
    for k, v in overrides.items():
        assert hasattr(conf, k), "unknown parameter " + k
        setattr(conf, k, v)
    return conf


@contextmanager
def template_add_contact():
    '''Temporarily restore add_contact_RF/LF of the template (without the fix of Q1), to reproduce the fall.'''
    def add_RF(self, transition_time=0.0):
        self.contactRF.setReference(self.robot.framePosition(self.formulation.data(), self.RF))
        self.formulation.addRigidContact(self.contactRF, self.conf.w_forceRef, self.conf.w_contact, 1)
        self.contact_RF_active = True

    def add_LF(self, transition_time=0.0):
        self.contactLF.setReference(self.robot.framePosition(self.formulation.data(), self.LF))
        self.formulation.addRigidContact(self.contactLF, self.conf.w_forceRef, self.conf.w_contact, 1)
        self.contact_LF_active = True

    cls = tsid_biped.TsidBiped
    fixed = cls.add_contact_RF, cls.add_contact_LF
    cls.add_contact_RF, cls.add_contact_LF = add_RF, add_LF
    try:
        yield
    finally:
        cls.add_contact_RF, cls.add_contact_LF = fixed


def walk(label, **overrides):
    '''Simulate one walk, print its indicators and return (log, metrics, conf).'''
    conf = make_conf(**overrides)
    log = biped_walking.simulate_walking(conf, viewer=None, verbose=False)
    m = biped_walking.compute_metrics(log, conf)
    fail = "" if log["fail"] is None else "  (stopped at t = %.2f s)" % log["fail"]
    print("\n=== %s%s" % (label, fail))
    biped_walking.print_metrics(m)
    return log, m, conf


def time_axis(log):
    return np.arange(log["N"]) * log["dt"]


def shade_phases(ax, log, phase="single", color="grey"):
    '''Grey background on the single (or double) support phases of the reference.'''
    t = time_axis(log)
    ph = np.array(log["contact_phase"][:log["N"]])
    where = ph != "double" if phase == "single" else ph == "double"
    ax.fill_between(t, 0, 1, where=where, color=color, alpha=0.15, transform=ax.get_xaxis_transform(), linewidth=0)


def ask_save():
    '''Ask on the terminal whether to save the figures (default: no).'''
    try:
        return input("\nSave the figures in %s (files with the same name are overwritten)? y/[N]: "
                     % FIG_DIR).strip().lower() == "y"
    except EOFError:
        return False


def save(figures):
    os.makedirs(FIG_DIR, exist_ok=True)
    for name, fig in figures.items():
        path = os.path.join(FIG_DIR, name + ".png")
        fig.savefig(path, dpi=150, bbox_inches="tight")
        print("Saved", os.path.relpath(path, HERE))


def legend_out(ax):
    ax.legend(loc="upper left", bbox_to_anchor=(1.01, 1.0))


# ------------------------------------------------------------------------------------------------
# Q1 - double support and contact force ramp
# ------------------------------------------------------------------------------------------------
def fig_cop_forces(log, title):
    '''Lateral reference CoP vs feet, and normal forces of the two feet over the whole walk.'''
    t, N = time_axis(log), log["N"]
    fig, ax = plut.create_empty_figure(2, 1)
    fig.set_size_inches(12, 7)
    ax[0].plot(t, log["cop_ref"][1, :N], "g", label="CoP ref y")
    ax[0].plot(t, log["com"][1, :N], "C3", label="CoM y")
    ax[0].set_ylabel("y [m]")
    ax[1].plot(t, log["f_RF"][2, :N], label="RF normal force")
    ax[1].plot(t, log["f_LF"][2, :N], label="LF normal force")
    ax[1].set_ylabel("Force [N]")
    ax[1].set_xlabel("Time [s]")
    for a in ax:
        shade_phases(a, log)
        legend_out(a)
    fig.suptitle(title + " (grey: single support)")
    return fig


def fig_switch_zoom(log_ds, log_nods, t_window=(2.3, 2.9)):
    '''Zoom on the first foot switch: reference CoP and normal forces, with and without double support.'''
    fig, ax = plut.create_empty_figure(2, 1)
    fig.set_size_inches(12, 7)
    for log, lab, st in [(log_ds, "with DS", "-"), (log_nods, "without DS", "--")]:
        t, N = time_axis(log), log["N"]
        ax[0].plot(t, log["cop_ref"][1, :N], st, color="g", label="CoP ref y, " + lab)
        ax[1].plot(t, log["f_RF"][2, :N], st, color="C0", label="RF, " + lab)
        ax[1].plot(t, log["f_LF"][2, :N], st, color="C1", label="LF, " + lab)
    for a in ax:
        shade_phases(a, log_ds, phase="double")
        a.set_xlim(*t_window)
        legend_out(a)
    ax[0].set_ylabel("CoP ref y [m]")
    ax[1].set_ylabel("Normal force [N]")
    ax[1].set_xlabel("Time [s]")
    fig.suptitle("First foot switch (grey: double support phase of the walk with DS)")
    return fig


def fig_forces_fmax(log, title, t_max=None):
    '''Normal forces and maximum normal forces of the two feet (fMax clipped at 1000 N).'''
    t, N = time_axis(log), log["N"]
    fig, ax = plut.create_empty_figure(2, 1)
    fig.set_size_inches(12, 7)
    ax[0].plot(t, log["f_RF"][2, :N], label="RF normal force")
    ax[0].plot(t, log["f_LF"][2, :N], label="LF normal force")
    ax[0].set_ylabel("Force [N]")
    ax[1].plot(t, np.minimum(log["fMax_RF"][:N], 1e3), label="RF max normal force")
    ax[1].plot(t, np.minimum(log["fMax_LF"][:N], 1e3), "--", label="LF max normal force")
    ax[1].set_ylabel("Max force [N]\n(clipped at 1000)")
    ax[1].set_xlabel("Time [s]")
    for a in ax:
        shade_phases(a, log)
        legend_out(a)
        if t_max is not None:
            a.set_xlim(0, t_max)
    if log["fail"] is not None:
        for a in ax:
            a.axvline(log["fail"], color="r", linestyle=":")
        title += ", fall detected at t = %.2f s (red line)" % log["fail"]
    fig.suptitle(title + " (grey: single support)")
    return fig


def fig_ramp_zoom(log_tt0, log_tt01, t_window=(2.3, 2.9)):
    '''Zoom on the first lift-off: normal and max normal force of the lifting foot, without/with ramp.'''
    fig, ax = plut.create_empty_figure(2, 1)
    fig.set_size_inches(12, 7)
    for log, lab, st in [(log_tt0, "transition_time = 0", "--"), (log_tt01, "transition_time = 0.1", "-")]:
        t, N = time_axis(log), log["N"]
        ax[0].plot(t, log["f_RF"][2, :N], st, color="C0", label="RF, " + lab)
        ax[0].plot(t, log["f_LF"][2, :N], st, color="C1", label="LF, " + lab)
        ax[1].plot(t, np.minimum(log["fMax_RF"][:N], 1e3), st, color="C0", label="RF max, " + lab)
    for a in ax:
        shade_phases(a, log_tt0, phase="double")
        a.set_xlim(*t_window)
        legend_out(a)
    ax[0].set_ylabel("Normal force [N]")
    ax[1].set_ylabel("Max normal force [N]\n(clipped at 1000)")
    ax[1].set_xlabel("Time [s]")
    fig.suptitle("First lift-off of the right foot (grey: double support phase)")
    return fig


def q1():
    log_ds, _, _ = walk("Q1 with double support (default)")
    log_nods, _, _ = walk("Q1 without double support", DATA_FILE_TSID="romeo_walking_no_ds.npz")
    with template_add_contact():
        log_fall, _, _ = walk("Q1 transition_time = 0.1, template add_contact (no fix)", transition_time=0.1)
    log_fix, _, _ = walk("Q1 transition_time = 0.1, with fix", transition_time=0.1)
    log_nods_ramp, _, _ = walk("Q1 transition_time = 0.1, with fix, without double support",
                               transition_time=0.1, DATA_FILE_TSID="romeo_walking_no_ds.npz")

    # weight transfer details printed for the report
    mg = sum(i.mass for i in log_ds["tsid_biped"].model.inertias) * 9.81
    print("\nRobot weight m*g = %.1f N" % mg)
    for log, lab in [(log_ds, "with DS"), (log_nods, "without DS"), (log_fix, "with DS, tt = 0.1")]:
        f = np.vstack([log["f_RF"][2, :log["N"]], log["f_LF"][2, :log["N"]]])
        d = np.abs(np.diff(f, axis=1))
        j = np.unravel_index(np.argmax(d), d.shape)
        print("%-18s largest one-step change of a normal force: %.1f N at t = %.3f s" % (lab, d[j], j[1] * log["dt"]))

    figures = {
        "q1_cop_forces_with_ds": fig_cop_forces(log_ds, "Walk with double support"),
        "q1_cop_forces_without_ds": fig_cop_forces(log_nods, "Walk without double support"),
        "q1_switch_zoom_ds_vs_no_ds": fig_switch_zoom(log_ds, log_nods),
        "q1_ramp_fall_template": fig_forces_fmax(log_fall, "transition_time = 0.1, template code", t_max=4.0),
        "q1_ramp_fixed": fig_forces_fmax(log_fix, "transition_time = 0.1, with fix"),
        "q1_ramp_zoom_liftoff": fig_ramp_zoom(log_ds, log_fix),
    }
    return figures


# ------------------------------------------------------------------------------------------------
# Q2 - tray moved along a vertical sinusoid while standing
# ------------------------------------------------------------------------------------------------
def tray(frequency, feedforward, verbose=True):
    conf = make_conf(tray=True, w_hand=1.0)   # as in tray_sinusoid.py
    r = tray_sinusoid.simulate_tray_sinusoid(conf, frequency, feedforward)
    if verbose:
        print("\n=== Q2 tray sinusoid")
        tray_sinusoid.print_tray_sinusoid(r)
    return r, conf


def pd_only_response(frequency, kp):
    '''Hand/reference transfer function without feedforward: p'' = Kp (p_ref - p) - Kd p'  ->  Kp / (s^2 + Kd s + Kp).'''
    s = 1j * 2 * np.pi * np.atleast_1d(np.asarray(frequency, dtype=float))
    H = kp / (s**2 + 2 * np.sqrt(kp) * s + kp)
    ratio, lag = np.abs(H), -np.degrees(np.unwrap(np.angle(H)))
    return (ratio[0], lag[0]) if np.ndim(frequency) == 0 else (ratio, lag)


def q2():
    runs = {}
    for f in [0.5, 2.0]:
        for ff in [True, False]:
            runs[(f, ff)], conf = tray(f, ff)
    kp = conf.kp_hand
    for f in [0.5, 2.0]:
        ratio, lag = pd_only_response(f, kp)
        print("theory without feedforward, f = %.1f Hz: amplitude ratio %.2f, phase lag %.1f deg" % (f, ratio, lag))
    for f in [0.5, 2.0]:
        print("required hand acceleration amplitude at %.1f Hz: %.2f m/s^2" % (f, 0.05 * (2 * np.pi * f)**2))

    # frequency sweep (validation of the explanation)
    freqs = [0.25, 0.5, 1.0, 1.5, 2.0, 2.5]
    sweep = {ff: [tray(f, ff, verbose=False)[0] for f in freqs] for ff in [True, False]}
    print("\nfrequency sweep:  f [Hz] | ratio/lag with FF | ratio/lag without FF | max CoM displ. with FF [mm]")
    for i, f in enumerate(freqs):
        a, b = sweep[True][i], sweep[False][i]
        print("  %4.2f | %.2f / %5.1f | %.2f / %5.1f | %5.1f" % (f, a["ratio"], a["lag"], b["ratio"], b["lag"], 1e3 * a["com_disp_max"]))

    # hand height tracking, 4 cases
    fig1, ax = plut.create_empty_figure(2, 1)
    fig1.set_size_inches(12, 7)
    for a, f in zip(ax, [0.5, 2.0]):
        r = runs[(f, True)]
        z0 = r["hand_ref"][2, 0]
        a.plot(r["t"], 1e3 * (r["hand_ref"][2] - z0), "k:", label="reference")
        a.plot(r["t"], 1e3 * (r["hand"][2] - z0), "C0", label="hand, with feedforward")
        a.plot(r["t"], 1e3 * (runs[(f, False)]["hand"][2] - z0), "C3--", label="hand, without feedforward")
        a.set_ylabel("hand z - z0 [mm]\n%.1f Hz" % f)
        legend_out(a)
    ax[1].set_xlabel("Time [s]")
    fig1.suptitle("Tray height (sinusoid of amplitude 5 cm)")

    # frequency response: simulation vs PD-only theory
    fs = np.linspace(0.1, 2.6, 200)
    th_ratio, th_lag = pd_only_response(fs, kp)
    fig2, ax = plut.create_empty_figure(2, 1)
    fig2.set_size_inches(12, 7)
    ax[0].plot(fs, th_ratio, "C3", label="theory without FF: $K_p/(s^2+K_ds+K_p)$")
    ax[0].plot(freqs, [r["ratio"] for r in sweep[False]], "C3o", label="simulation without FF")
    ax[0].plot(freqs, [r["ratio"] for r in sweep[True]], "C0s", label="simulation with FF")
    ax[0].axhline(1.0, color="C0", linestyle=":", label="ideal with FF")
    ax[0].set_ylabel("Amplitude ratio")
    ax[1].plot(fs, th_lag, "C3", label="theory without FF")
    ax[1].plot(freqs, [r["lag"] for r in sweep[False]], "C3o", label="simulation without FF")
    ax[1].plot(freqs, [r["lag"] for r in sweep[True]], "C0s", label="simulation with FF")
    ax[1].axhline(0.0, color="C0", linestyle=":", label="ideal with FF")
    ax[1].set_ylabel("Phase lag [deg]")
    ax[1].set_xlabel("Frequency [Hz]")
    for a in ax:
        a.axvline(np.sqrt(kp) / (2 * np.pi), color="grey", linestyle="--")
        legend_out(a)
    fig2.suptitle("Hand/reference frequency response ($K_p$ = %.0f, dashed: $\\omega_n = \\sqrt{K_p}$)" % kp)

    # CoM displacement with feedforward
    fig3, ax = plut.create_empty_figure(2, 1)
    fig3.set_size_inches(12, 7)
    for a, f in zip(ax, [0.5, 2.0]):
        r = runs[(f, True)]
        for i, lab in enumerate("xyz"):
            a.plot(r["t"], 1e3 * (r["com"][i] - r["com_0"][i]), label="CoM " + lab)
        a.set_ylabel("CoM displacement [mm]\n%.1f Hz, with FF" % f)
        legend_out(a)
    ax[1].set_xlabel("Time [s]")
    fig3.suptitle("CoM displacement while moving the tray with feedforward")

    return {"q2_hand_height_tracking": fig1, "q2_frequency_response": fig2, "q2_com_displacement_ff": fig3}


# ------------------------------------------------------------------------------------------------
# Q3 - walking with the tray
# ------------------------------------------------------------------------------------------------
TRAY_WALK = dict(tray=True, transition_time=0.1)   # Part 2: default reference file, ramp of Q1 with the fix


def com_error(log):
    '''Horizontal CoM tracking error (x, y) during the walk.'''
    N = log["N"]
    return log["com"][:2, :N] - log["com_ref"][:2, :N]


def q3():
    log0, m0, _ = walk("Q3 tray, without hand task (w_hand = 0)", w_hand=0.0, **TRAY_WALK)
    log1, m1, _ = walk("Q3 tray, with hand task (w_hand = 1)", w_hand=1.0, **TRAY_WALK)
    for log, lab in [(log0, "w_hand = 0"), (log1, "w_hand = 1")]:
        e = com_error(log)
        print("%s: CoM error RMS x %.1f mm, y %.1f mm | max x %.1f mm, y %.1f mm" % (
            lab, 1e3 * np.sqrt(np.mean(e[0]**2)), 1e3 * np.sqrt(np.mean(e[1]**2)),
            1e3 * np.max(np.abs(e[0])), 1e3 * np.max(np.abs(e[1]))))

    t = time_axis(log0)
    N = log0["N"]
    fig1, ax = plut.create_empty_figure(3, 1)
    fig1.set_size_inches(12, 9)
    for log, lab, st in [(log0, "w_hand = 0", "C3--"), (log1, "w_hand = 1", "C0")]:
        ax[0].plot(t, log["tray_tilt"][:N], st, label=lab)
        ax[1].plot(t, 1e3 * (log["hand"][2, :N] - log["hand_ref"][2, 0]), st, label=lab)
        ax[2].plot(t, 1e3 * np.linalg.norm(log["hand"][:, :N] - log["hand_ref"][:, :N], axis=0), st, label=lab)
    ax[0].set_ylabel("Tray tilt [deg]")
    ax[1].set_ylabel("Hand height\nchange [mm]")
    ax[2].set_ylabel("Hand position\nerror [mm]")
    ax[2].set_xlabel("Time [s]")
    for a in ax:
        shade_phases(a, log0)
        legend_out(a)
    fig1.suptitle("Walking with the tray: tray tilt and hand tracking (grey: single support)")

    fig2, ax = plut.create_empty_figure(2, 1)
    fig2.set_size_inches(12, 7)
    for log, lab, st in [(log0, "w_hand = 0", "C3--"), (log1, "w_hand = 1", "C0")]:
        e = com_error(log)
        ax[0].plot(t, 1e3 * e[0], st, label=lab)
        ax[1].plot(t, 1e3 * e[1], st, label=lab)
    ax[0].set_ylabel("CoM error x [mm]")
    ax[1].set_ylabel("CoM error y [mm]")
    ax[1].set_xlabel("Time [s]")
    for a in ax:
        shade_phases(a, log0)
        legend_out(a)
    fig2.suptitle("CoM tracking error with and without the tray task (grey: single support)")
    return {"q3_tray_tilt_hand_tracking": fig1, "q3_com_error": fig2}


# ------------------------------------------------------------------------------------------------
# Q4 - mask and frame of the tray task
# ------------------------------------------------------------------------------------------------
Q4_CASES = [  # (label, mask [x, y, z, rot_x, rot_y, rot_z], local frame)
    ("full mask, local", [1, 1, 1, 1, 1, 1], True),
    ("full mask, world", [1, 1, 1, 1, 1, 1], False),
    ("position only, local", [1, 1, 1, 0, 0, 0], True),
    ("orientation only, local", [0, 0, 0, 1, 1, 1], True),
    ("waiter, local", [0, 0, 1, 1, 1, 0], True),
    ("waiter, world", [0, 0, 1, 1, 1, 0], False),
]


def q4():
    logs, mets = {}, {}
    for lab, mask, local in Q4_CASES:
        logs[lab], mets[lab], conf = walk("Q4 " + lab, w_hand=1.0, hand_mask=np.array(mask, dtype=float),
                                          hand_local_frame=local, **TRAY_WALK)
    # orientation of the hand frame w.r.t. the world at the beginning (to interpret the local frame)
    R = logs["full mask, local"]["tsid_biped"].H_hand_0.rotation
    print("\nInitial hand frame axes (world coordinates) and angle with the world vertical:")
    for k, a in enumerate("xyz"):
        print("  hand %s = %s, %.1f deg from the vertical" % (a, np.round(R[:, k], 3), np.degrees(np.arccos(R[2, k]))))

    shown = ["position only, local", "orientation only, local", "waiter, local", "waiter, world"]
    styles = ["C1", "C2", "C3--", "C0"]
    t = time_axis(logs[shown[0]])
    N = logs[shown[0]]["N"]
    fig1, ax = plut.create_empty_figure(2, 1)
    fig1.set_size_inches(12, 7)
    for lab, st in zip(shown, styles):
        log = logs[lab]
        ax[0].plot(t, log["tray_tilt"][:N], st, label=lab)
        ax[1].plot(t, 1e3 * (log["hand"][2, :N] - log["hand_ref"][2, 0]), st, label=lab)
    ax[0].set_ylabel("Tray tilt [deg]")
    ax[1].set_ylabel("Hand height change [mm]")
    ax[1].set_xlabel("Time [s]")
    for a in ax:
        shade_phases(a, logs[shown[0]])
        legend_out(a)
    fig1.suptitle("Tray task with different masks and frames (grey: single support)")

    labels = [c[0] for c in Q4_CASES]
    vals = [("Max tray tilt [deg]", [mets[l]["tray_tilt_max"] for l in labels]),
            ("Hand height range [mm]", [1e3 * mets[l]["hand_z_range"] for l in labels]),
            ("CoM error RMS [mm]", [1e3 * mets[l]["com_err_rms"] for l in labels])]
    fig2, ax = plt.subplots(1, 3, figsize=(15, 5))
    for a, (name, v) in zip(ax, vals):
        bars = a.barh(range(len(labels)), v, color=["C7", "C7", "C1", "C2", "C3", "C0"])
        a.set_yticks(range(len(labels)))
        a.set_yticklabels(labels if a is ax[0] else [])
        a.invert_yaxis()
        a.set_title(name)
        a.grid(True, axis="x")
        for b, x in zip(bars, v):
            a.text(b.get_width(), b.get_y() + b.get_height() / 2, " %.2f" % x if x < 1 else " %.1f" % x,
                   va="center", fontsize=10)
    fig2.suptitle("Summary of the tray task variants (waiter mask = [0, 0, 1, 1, 1, 0])")
    fig2.tight_layout()
    return {"q4_masks_tilt_height": fig1, "q4_masks_summary": fig2}


# ------------------------------------------------------------------------------------------------
# Q5 - heavy trays
# ------------------------------------------------------------------------------------------------
def fig_normalized_torques(logs, title, threshold=0.8):
    '''Normalized torques |tau|/tau_max of the joints that exceed the threshold in at least one log.'''
    fig, ax = plut.create_empty_figure(len(logs), 1)
    fig.set_size_inches(12, 3.5 * len(logs))
    ax = np.atleast_1d(ax)
    for a, (lab, log) in zip(ax, logs.items()):
        N = log["N"]
        stop = N if log["fail"] is None else int(round(log["fail"] / log["dt"]))
        t = time_axis(log)[:stop]
        for j, name in enumerate(log["joint_names"]):
            r = log["tau"][j, :stop] / log["tau_max"][j]
            if np.max(np.abs(r)) > threshold:
                a.plot(t, r, label=name)
        a.axhline(1.0, color="k", linestyle=":")
        a.axhline(-1.0, color="k", linestyle=":")
        if log["fail"] is not None:
            a.axvline(log["fail"], color="r", linestyle="--", label="QP infeasible")
        a.set_ylabel("$\\tau/\\tau_{max}$\n" + lab)
        a.set_xlim(0, time_axis(log)[-1])
        shade_phases(a, log)
        legend_out(a)
    ax[-1].set_xlabel("Time [s]")
    fig.suptitle(title + " (joints above %d%% of the limit; grey: single support)" % (100 * threshold))
    return fig


def q5():
    import compare_payloads
    import pinocchio as pin

    # Part A: 5 kg tray (torque limits x4) vs empty tray, as in compare_payloads.py
    runs = {k: compare_payloads.simulate(m, compare_payloads.TAU_MAX_SCALING[k])
            for k, m in compare_payloads.PAYLOADS.items()}
    n = min(len(runs["solid"][1]), len(runs["ghost"][1]))
    dt = runs["solid"][0].dt
    t = np.arange(n) * dt - runs["solid"][0].T_pre
    Q5, com5 = runs["solid"][1][:n], runs["solid"][2][:n]
    Q0, com0 = runs["ghost"][1][:n], runs["ghost"][2][:n]
    walking = t >= 0
    print("\n=== Q5 part A: 5 kg tray vs empty tray (differences 5 kg - empty, during the walk)")
    for i, a in enumerate("xyz"):
        d_pel, d_com = Q5[walking, i] - Q0[walking, i], com5[walking, i] - com0[walking, i]
        print("  %s: pelvis mean %+6.1f mm (min %+6.1f, max %+6.1f) | total CoM mean %+5.1f mm (max |.| %4.1f)" % (
            a, 1e3 * d_pel.mean(), 1e3 * d_pel.min(), 1e3 * d_pel.max(), 1e3 * d_com.mean(), 1e3 * np.abs(d_com).max()))
    # CoM of the robot alone (without payload) and position of the hand, to explain the pelvis shift
    robot = pin.RobotWrapper.BuildFromURDF(romeo_conf.urdf, [romeo_conf.path], pin.JointModelFreeFlyer())
    m_r, d_r = robot.model, robot.model.createData()
    hand_id = m_r.getFrameId(romeo_conf.hand_frame_name)
    def robot_com_and_hand(q):
        pin.framesForwardKinematics(m_r, d_r, q)
        return pin.centerOfMass(m_r, d_r, q).copy(), d_r.oMf[hand_id].translation.copy()
    c5, h5 = zip(*[robot_com_and_hand(q) for q in Q5[walking][::25]])
    c0, h0 = zip(*[robot_com_and_hand(q) for q in Q0[walking][::25]])
    print("  robot-only CoM (5 kg - empty) mean: %s mm" % np.round(1e3 * (np.mean(c5, 0) - np.mean(c0, 0)), 1))
    print("  hand position relative to the pelvis, mean (5 kg): %s mm, (empty): %s mm" % (
        np.round(1e3 * np.mean(np.array(h5) - Q5[walking][::25, :3], 0), 1),
        np.round(1e3 * np.mean(np.array(h0) - Q0[walking][::25, :3], 0), 1)))
    print("  payload lever: 5 kg tray at %.0f mm lateral from the pelvis -> moment %.1f Nm" % (
        1e3 * np.mean(np.array(h5)[:, 1] - Q5[walking][::25, 1]), 5 * 9.81 * np.mean(np.array(h5)[:, 1] - Q5[walking][::25, 1])))

    fig1, ax = plut.create_empty_figure(2, 1)
    fig1.set_size_inches(12, 7)
    for i, a in zip([1, 0], ax):
        lab = "lateral (y)" if i == 1 else "forward (x)"
        a.plot(t, 1e3 * Q5[:, i], "C0", label="pelvis, 5 kg tray")
        a.plot(t, 1e3 * Q0[:, i], "C0--", label="pelvis, empty tray")
        a.plot(t, 1e3 * com5[:, i], "C3", label="total CoM, 5 kg tray")
        a.plot(t, 1e3 * com0[:, i], "C3--", label="total CoM, empty tray")
        a.set_ylabel("%s position [mm]" % lab)
        a.grid(True)
        legend_out(a)
    ax[1].set_xlabel("Time [s] (0 = start of the walk)")
    fig1.suptitle("Heavy tray: pelvis and total CoM (robot + tray)")

    # Part B: 2 kg tray, nominal torque limits, weighted hand task vs hard constraint
    logs, mets = {}, {}
    for lab, ov in [("w_hand = 100", dict(w_hand=100.0)),
                    ("hard constraint", dict(w_hand=100.0, hand_priority=0))]:
        logs[lab], mets[lab], _ = walk("Q5 part B: 2 kg tray, " + lab, payload_mass=2.0, **TRAY_WALK, **ov)
    # empty tray with w_hand = 100 as a reference for the tray indicators
    _, m_empty, _ = walk("Q5 part B reference: empty tray, w_hand = 100", w_hand=100.0, **TRAY_WALK)
    log_w = logs["w_hand = 100"]
    names = log_w["joint_names"]
    sat = np.abs(log_w["tau"][:, :log_w["N"]]) / log_w["tau_max"][:, None]
    for j in np.argsort(-sat.max(1))[:4]:
        print("  w_hand = 100: %-15s max |tau|/tau_max %.2f, time at the limit %.2f s" % (
            names[j], sat[j].max(), np.sum(sat[j] > 0.99) * log_w["dt"]))
    log_h = logs["hard constraint"]
    if log_h["fail"] is not None:
        k = int(round(log_h["fail"] / log_h["dt"])) - 1
        r = np.abs(log_h["tau"][:, k]) / log_h["tau_max"]
        print("  hard constraint: infeasible at t = %.3f s; joints at the limit just before: %s" % (
            log_h["fail"], ", ".join("%s (%.2f)" % (names[j], r[j]) for j in np.argsort(-r)[:3])))

    fig2 = fig_normalized_torques({"2 kg, w_hand = 100": log_w, "2 kg, hard constraint": log_h},
                                  "Normalized joint torques with a 2 kg tray")
    fig3, ax = plut.create_empty_figure(2, 1)
    fig3.set_size_inches(12, 7)
    tt = time_axis(log_w)
    N = log_w["N"]
    ax[0].plot(tt, log_w["tray_tilt"][:N], "C0", label="2 kg, w_hand = 100")
    ax[1].plot(tt, 1e3 * (log_w["hand"][2, :N] - log_w["hand_ref"][2, 0]), "C0", label="2 kg, w_hand = 100")
    ax[0].set_ylabel("Tray tilt [deg]")
    ax[1].set_ylabel("Hand height change [mm]")
    ax[1].set_xlabel("Time [s]")
    for a in ax:
        shade_phases(a, log_w)
        legend_out(a)
    fig3.suptitle("2 kg tray with the weighted hand task (grey: single support)")
    print("\n  tray indicators: empty tray w_hand=100: tilt %.2f deg, height range %.1f mm | 2 kg w_hand=100: tilt %.2f deg, height range %.1f mm, hand err max %.1f mm" % (
        m_empty["tray_tilt_max"], 1e3 * m_empty["hand_z_range"], mets["w_hand = 100"]["tray_tilt_max"],
        1e3 * mets["w_hand = 100"]["hand_z_range"], 1e3 * mets["w_hand = 100"]["hand_err_max"]))
    return {"q5_heavy_tray_pelvis_com": fig1, "q5_torques_2kg_weighted_vs_hard": fig2, "q5_tray_2kg_weighted": fig3}


# ------------------------------------------------------------------------------------------------
if __name__ == "__main__":
    questions = {"q1": q1, "q2": q2, "q3": q3, "q4": q4, "q5": q5}
    if len(sys.argv) != 2 or sys.argv[1] not in questions:
        sys.exit("usage: python3 experiments.py {%s}" % ",".join(questions))
    figures = questions[sys.argv[1]]()
    if ask_save():
        save(figures)
    plt.show()
