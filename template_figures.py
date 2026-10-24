"""
Figures of the report produced with the plotting code of the templates (biped_walking.plot_walking and the
__main__ of tray_sinusoid.py and compare_payloads.py): one simulation per figure, no overlays.

The templates are NOT modified. For every case this script does what the assignment asks to do by hand:
it sets the parameters of romeo_conf.py needed by the question (the default values are restored before the
next case), runs the template, prints its indicators and saves the figures it produces.

    python3 template_figures.py q1            # q1, q2, q3, q4, q5, or: all
    python3 template_figures.py q1 q3

At launch the script asks (once) whether the figures must be saved, whatever the questions selected.
Figures are saved as PDF in figures/template/.
The legends may be also moved outside the plots.
Question 5, part A (compare_payloads.py) also needs meshcat, but no browser: the viewer is not opened here.
"""
import importlib
import os
import sys
from contextlib import contextmanager

import matplotlib
matplotlib.use("Agg")                       # no windows: the figures are only saved
import matplotlib.pyplot as plt
import numpy as np

import biped_walking
import plot_utils as plut
import romeo_conf
import tray_sinusoid
import tsid_biped

plt.rcParams["pdf.fonttype"] = 42           # embed fonts as TrueType in the PDF files

HERE = os.path.dirname(os.path.abspath(__file__))
FIG_DIR = os.path.join(HERE, "figures", "template")
SAVE_FIGURES = True                         # set at launch by ask_save_figures()
TRAY_WALK = dict(tray=True, transition_time=0.1)   # part 2: default reference file, ramp of question 1 with the fix

# figures created by biped_walking.plot_walking, in this order
FIGS_NO_TRAY = ["com", "forces", "torques"]
FIGS_TRAY = ["com", "forces", "hand", "torques"]


# ------------------------------------------------------------------------------------------------
# helpers
# ------------------------------------------------------------------------------------------------
def make_conf(**overrides):
    '''Default parameters of romeo_conf.py (reloaded), without viewer, plus the given overrides.'''
    conf = importlib.reload(romeo_conf)
    conf.viewer = None
    for k, v in overrides.items():
        assert hasattr(conf, k), "unknown parameter " + k
        setattr(conf, k, v)
    return conf


@contextmanager
def template_add_contact():
    '''Temporarily restore add_contact_RF/LF of the template (without the fix of question 1), to reproduce the fall.'''
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


def legends_outside(fig):
    '''Move the legend of every axes of the figure outside the plot, on the right.'''
    for ax in fig.axes:
        if ax.get_legend() is None:
            continue
        handles, labels = ax.get_legend_handles_labels()
        ax.legend(handles, labels, loc="upper left", bbox_to_anchor=(1.01, 1.0), borderaxespad=0.0)


def ask_save_figures():
    '''Ask the if figures must be saved. '''
    global SAVE_FIGURES
    try:
        answer = input("Save the figures in figures/template/? [Y/n]: ").strip().lower()
    except EOFError:                         # no terminal: keep the default
        answer = ""
    SAVE_FIGURES = answer not in ("n", "no")
    print("Figures will %sbe saved." % ("" if SAVE_FIGURES else "NOT "))


def save_new_figures(before, names):
    '''Save the figures created since `before` (in creation order) with the given names; None = do not save.
       If the user chose not to save the figures, they are only closed.'''
    new = [n for n in plt.get_fignums() if n not in before]
    assert len(new) == len(names), "expected %d figures, found %d" % (len(names), len(new))
    if SAVE_FIGURES:
        os.makedirs(FIG_DIR, exist_ok=True)
    for num, name in zip(new, names):
        # legends_outside(plt.figure(num)) doesn't look good 
        if name is None or not SAVE_FIGURES:
            continue
        path = os.path.join(FIG_DIR, name + ".pdf")
        plt.figure(num).savefig(path, bbox_inches="tight")
        print("Saved", os.path.relpath(path, HERE))
    plt.close("all")


def walk(label, saved, tray=False, **overrides):
    '''biped_walking.py: simulate, print the indicators and plot (as its __main__). saved = names of the files
       for the figures of plot_walking, in order (com, forces, [hand,] torques); None = do not save.'''
    conf = make_conf(tray=tray, **overrides)
    kinds = FIGS_TRAY if conf.tray else FIGS_NO_TRAY
    names = [saved.get(k) for k in kinds]
    before = set(plt.get_fignums())
    print("\n=== " + label)
    log = biped_walking.simulate_walking(conf, viewer=None, verbose=False)
    metrics = biped_walking.compute_metrics(log, conf)
    biped_walking.print_metrics(metrics)
    biped_walking.plot_walking(log, conf)
    save_new_figures(before, names)
    return metrics


# ------------------------------------------------------------------------------------------------
# questions
# ------------------------------------------------------------------------------------------------
def q1():
    walk("Q1 default reference file (double support)", dict(com="q1_ds_com", forces="q1_ds_forces"))
    walk("Q1 without double support", dict(com="q1_nods_com", forces="q1_nods_forces"),
         DATA_FILE_TSID="romeo_walking_no_ds.npz")
    with template_add_contact():
        walk("Q1 transition_time = 0.1, code of the template (the robot falls)", dict(forces="q1_fall_forces"),
             transition_time=0.1)
    walk("Q1 transition_time = 0.1, with the fix", dict(forces="q1_fix_forces"), transition_time=0.1)


def q2():
    '''tray_sinusoid.py (its __main__ with the given FREQUENCY and FEEDFORWARD).'''
    for frequency, ff, f_name, ff_name in [(0.5, True, "0p5hz", "ff"), (0.5, False, "0p5hz", "noff"),
                                           (2.0, True, "2hz", "ff"), (2.0, False, "2hz", "noff")]:
        conf = make_conf(tray=True, w_hand=1.0)             # as in tray_sinusoid.py
        before = set(plt.get_fignums())
        print("\n=== Q2 tray sinusoid, %.1f Hz, feedforward %s" % (frequency, ff))
        r = tray_sinusoid.simulate_tray_sinusoid(conf, frequency, ff, tray_sinusoid.AMPLITUDE,
                                                 tray_sinusoid.DURATION, None)
        tray_sinusoid.print_tray_sinusoid(r)
        # plots: copied from the __main__ of tray_sinusoid.py
        t_log, hand, hand_ref, com, com_0, tray_tilt = (r[k] for k in ["t", "hand", "hand_ref", "com", "com_0", "tray_tilt"])
        (f, ax) = plut.create_empty_figure(3, 1)
        for i, lab in enumerate(["x", "y", "z"]):
            ax[i].plot(t_log, hand[i], label="hand " + lab)
            ax[i].plot(t_log, hand_ref[i], "r:", label="hand ref " + lab)
            ax[i].set_ylabel("[m]")
            ax[i].legend()
        ax[2].set_xlabel("Time [s]")
        (f, ax) = plut.create_empty_figure(2, 1)
        for i, lab in enumerate(["x", "y"]):
            ax[0].plot(t_log, com[i] - com_0[i], label="CoM " + lab)
        ax[0].plot(t_log, com[2] - com_0[2], label="CoM z")
        ax[0].set_ylabel("CoM displacement [m]")
        ax[0].legend()
        ax[1].plot(t_log, tray_tilt, label="tray tilt")
        ax[1].set_ylabel("[deg]")
        ax[1].set_xlabel("Time [s]")
        ax[1].legend()
        tag = "q2_%s_%s" % (f_name, ff_name)
        save_new_figures(before, [tag + "_hand", tag + "_com" if ff else None])


def q3():
    for w_hand in [0.0, 1.0]:
        tag = "q3_whand%d" % w_hand
        walk("Q3 tray, w_hand = %g" % w_hand, dict(com=tag + "_com", hand=tag + "_hand"), w_hand=w_hand, **TRAY_WALK)


Q4_CASES = [  
    ''' Definition of the different masks of question 4, as well as the used frame. (name, mask [x, y, z, rot_x, rot_y, rot_z], local frame)'''
    ("q4_pose_local", [1, 1, 1, 1, 1, 1], True),
    ("q4_position_local", [1, 1, 1, 0, 0, 0], True),
    ("q4_attitude_local", [0, 0, 0, 1, 1, 1], True),
    ("q4_waiter_local", [0, 0, 1, 1, 1, 0], True),
    ("q4_waiter_world", [0, 0, 1, 1, 1, 0], False),
]


def q4():
    '''Masks of the tray task. Position-only and orientation-only are tested in the world-aligned frame;
       the waiter mask is tested in both frames. Indicators of all cases are summarised at the end.'''
    results = []
    for name, mask, local in Q4_CASES:
        m = walk("Q4 mask %s, local frame %s" % (mask, local), dict(hand=name + "_hand"), w_hand=1.0,
                 hand_mask=np.array(mask, dtype=float), hand_local_frame=local, **TRAY_WALK)
        results.append((name, m))
    print("\n=== Q4 summary")
    print("%-24s %10s %14s %16s %16s" % ("case", "completed", "tilt max [deg]", "hand z range [m]",
                                         "CoM err RMS [m]"))
    for name, m in results:
        print("%-24s %10s %14.2f %16.4f %16.4f" % (name, m["completed"], m["tray_tilt_max"], m["hand_z_range"],
                                                  m["com_err_rms"]))


def q5():
    # part A: compare_payloads.py (5 kg tray vs empty tray), plot copied from its __main__
    import compare_payloads
    print("\n=== Q5 part A: 5 kg tray vs empty tray")
    runs = {k: compare_payloads.simulate(m, compare_payloads.TAU_MAX_SCALING[k])
            for k, m in compare_payloads.PAYLOADS.items()}
    conf = runs["solid"][0]
    n = min(len(runs["solid"][1]), len(runs["ghost"][1]))
    before = set(plt.get_fignums())
    t = np.arange(n) * conf.dt - conf.T_pre
    fig, ax = plt.subplots(2, 1, sharex=True)
    for k, style in [("solid", "-"), ("ghost", "--")]:
        _, Q, com = runs[k]
        lab = "%.1f kg tray" % compare_payloads.PAYLOADS[k]
        ax[0].plot(t, 1e3 * Q[:n, 1], style, label="pelvis, " + lab)
        ax[1].plot(t, 1e3 * com[:n, 1], style, label="total CoM, " + lab)
    ax[0].plot(t, 1e3 * (runs["solid"][1][:n, 1] - runs["ghost"][1][:n, 1]), "k:", label="pelvis difference")
    ax[0].set_ylabel("lateral position [mm]")
    ax[1].set_ylabel("lateral position [mm]")
    ax[1].set_xlabel("Time [s]")
    for a in ax:
        a.legend()
        a.grid(True)
    fig.suptitle("Lateral position of pelvis and total CoM")
    save_new_figures(before, ["q5_payloads_lateral"])

    # part B: 2 kg tray, nominal torque limits, weighted task (w_hand = 100) and hard constraint (priority 0)
    walk("Q5 part B: 2 kg tray, w_hand = 100", dict(hand="q5_2kg_weighted_hand", torques="q5_2kg_weighted_torques"),
         payload_mass=2.0, w_hand=100.0, **TRAY_WALK)
    walk("Q5 part B: 2 kg tray, hand task as hard constraint (hand_priority = 0)",
         dict(torques="q5_2kg_hard_torques"), payload_mass=2.0, w_hand=100.0, hand_priority=0, **TRAY_WALK)


# ------------------------------------------------------------------------------------------------
if __name__ == "__main__":
    questions = {"q1": q1, "q2": q2, "q3": q3, "q4": q4, "q5": q5}
    args = sys.argv[1:]
    selected = list(questions) if args == ["all"] else args
    if not selected or any(q not in questions for q in selected):
        sys.exit("usage: python3 template_figures.py {all,%s} [...]" % ",".join(questions))
    ask_save_figures()                     
    for q in selected:
        print("\n" + "#" * 25 + " " + q + " " + "#" * 25)
        questions[q]()
