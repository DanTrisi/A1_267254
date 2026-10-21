"""
Warm-up of the tray task (question 2): the robot stands on both feet and moves the tray along a sinusoidal
trajectory with the tray (hand) task, while keeping its orientation constant.

    python tray_sinusoid.py

Change FREQUENCY and FEEDFORWARD below to answer the question.
"""
import importlib
import time

import matplotlib.pyplot as plt
import numpy as np
import plot_utils as plut
import romeo_conf
from numpy.linalg import norm
from tsid_biped import TsidBiped

AMPLITUDE = np.array([0.0, 0.0, 0.05])  # amplitude of the sinusoid along x, y, z [m]
FREQUENCY = 0.5                         # frequency of the sinusoid [Hz]
FEEDFORWARD = True                      # if False, the velocity/acceleration references are set to zero
DURATION = 6.0                          # duration of the simulation [s]
T_SKIP = 2.0                            # initial transient, not used to compute the tracking indicators [s]


def sinusoid_reference(t, center, amplitude, frequency):
    '''
    Reference of the hand at time t for a sinusoidal motion around the position center:
        p(t) = center + amplitude * sin(2 pi frequency t)
    Returns position, velocity and acceleration (3d arrays).
    '''
    ### TODO ###
    w = 2 * np.pi * frequency
    pos = center + amplitude * np.sin(w * t)
    vel = amplitude * w * np.cos(w * t)
    acc = -amplitude * w**2 * np.sin(w * t)
    return pos, vel, acc


def amplitude_ratio_and_lag(t, x, x_ref, frequency):
    '''Fit a sinusoid of the given frequency to x and x_ref: return the ratio of their amplitudes
       and the phase lag of x with respect to x_ref [deg].'''
    w = 2 * np.pi * frequency
    S = np.column_stack([np.sin(w * t), np.cos(w * t), np.ones_like(t)])
    c = np.linalg.lstsq(S, x, rcond=None)[0]
    c_ref = np.linalg.lstsq(S, x_ref, rcond=None)[0]
    ratio = np.hypot(c[0], c[1]) / np.hypot(c_ref[0], c_ref[1])
    lag = np.degrees(np.arctan2(c_ref[1], c_ref[0]) - np.arctan2(c[1], c[0]))
    return ratio, (lag + 180.0) % 360.0 - 180.0


def simulate_tray_sinusoid(conf, frequency=FREQUENCY, feedforward=FEEDFORWARD, amplitude=AMPLITUDE,
                           duration=DURATION, viewer=None):
    '''Robot standing on both feet, moving the tray along a sinusoid. Returns a dictionary with the logs.'''
    tsid_biped = TsidBiped(conf, viewer)
    H_hand_0 = tsid_biped.H_hand_0
    com_0 = tsid_biped.robot.com(tsid_biped.formulation.data()).copy()
    tsid_biped.set_com_ref(com_0, 0 * com_0, 0 * com_0)

    N = int(duration / conf.dt)
    t_log = np.arange(N) * conf.dt
    hand, hand_ref = np.full((3, N), np.nan), np.full((3, N), np.nan)
    com, tray_tilt = np.full((3, N), np.nan), np.full(N, np.nan)

    if viewer:
        input("Wait for the robot to appear in the browser, then press enter to start")
    q, v = tsid_biped.q, tsid_biped.v
    t = 0.0
    for i in range(N):
        time_start = time.time()
        pos, vel, acc = sinusoid_reference(t, H_hand_0.translation, amplitude, frequency)
        if not feedforward:
            vel, acc = 0 * vel, 0 * acc
        tsid_biped.set_hand_ref(pos, H_hand_0.rotation, vel, acc)

        sol = tsid_biped.solver.solve(tsid_biped.formulation.computeProblemData(t, q, v))
        if sol.status != 0:
            print("Time %.3f QP problem could not be solved! Error code:" % t, sol.status)
            break
        dv = tsid_biped.formulation.getAccelerations(sol)

        H = tsid_biped.get_placement_hand()
        hand[:, i], hand_ref[:, i] = H.translation, pos
        com[:, i] = tsid_biped.robot.com(tsid_biped.formulation.data())
        R = H.rotation @ H_hand_0.rotation.T
        tray_tilt[i] = np.degrees(np.arccos(np.clip(R[2, 2], -1.0, 1.0)))

        q, v = tsid_biped.integrate_dv(q, v, dv, conf.dt)
        t += conf.dt
        if norm(v) > 20.0:
            print("Time %.3f Velocities are too high, stop everything!" % t)
            break
        if viewer and i % conf.DISPLAY_N == 0:
            tsid_biped.display(q)
            time_spent = time.time() - time_start
            if time_spent < conf.DISPLAY_N * conf.dt:
                time.sleep(conf.DISPLAY_N * conf.dt - time_spent)

    # tracking indicators, along the direction of the motion and after the initial transient
    k = int(T_SKIP / conf.dt)
    j = int(np.argmax(np.abs(amplitude)))
    if np.nanmax(np.abs(hand_ref[j, k:] - hand_ref[j, k])) < 1e-6:
        raise SystemExit("The reference of the hand does not move: complete sinusoid_reference()")
    ratio, lag = amplitude_ratio_and_lag(t_log[k:], hand[j, k:], hand_ref[j, k:], frequency)
    return dict(t=t_log, hand=hand, hand_ref=hand_ref, com=com, com_0=com_0, tray_tilt=tray_tilt,
                frequency=frequency, feedforward=feedforward, ratio=ratio, lag=lag,
                hand_err_rms=np.sqrt(np.nanmean(np.sum((hand[:, k:] - hand_ref[:, k:])**2, 0))),
                com_disp_max=np.nanmax(norm(com[:, k:] - com_0[:, None], axis=0)),
                tilt_max=np.nanmax(tray_tilt[k:]))


def print_tray_sinusoid(r):
    print("Frequency %.2f Hz, feedforward %s" % (r["frequency"], r["feedforward"]))
    print("Amplitude ratio (hand / reference): %.2f" % r["ratio"])
    print("Phase lag [deg]:                    %.1f" % r["lag"])
    print("Hand tracking error RMS [m]:        %.4f" % r["hand_err_rms"])
    print("Max CoM displacement [m]:           %.4f" % r["com_disp_max"])
    print("Max tray tilt [deg]:                %.2f" % r["tilt_max"])


if __name__ == "__main__":
    conf = importlib.reload(romeo_conf)
    conf.tray = True
    conf.w_hand = 1.0  # the tray task must be active
    r = simulate_tray_sinusoid(conf, FREQUENCY, FEEDFORWARD, AMPLITUDE, DURATION, conf.viewer)
    print_tray_sinusoid(r)
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
    plt.show()
