import os
import time

import matplotlib.pyplot as plt
import numpy as np
import plot_utils as plut
import romeo_conf
from numpy import nan
from numpy.linalg import norm
from tsid_biped import TsidBiped


def compute_contact_events(contact_phase, n_transition):
    '''
    Given the sequence of contact phases ('double', 'left', 'right') of the reference gait,
    compute when contacts must be created and when their removal must start.
    Returns two dictionaries {time-step index: 'LF' or 'RF'}:
     - make_contact : index at which the contact of a foot must be added (touchdown)
     - break_contact: index at which the removal of the contact of a foot must start, so that
                      the force ramp (n_transition time steps long) ends exactly at lift-off,
                      i.e. when the swing foot reference starts moving.
    '''
    make_contact, break_contact = {}, {}
    for i in range(1, len(contact_phase)):
        prev, cur = contact_phase[i - 1], contact_phase[i]
        if prev == cur:
            continue
        if prev in ('left', 'right'):  # end of single support: the swing foot touches down
            make_contact[i] = 'LF' if prev == 'right' else 'RF'
        if cur in ('left', 'right'):  # start of single support: the other foot lifts off
            break_contact[i - n_transition] = 'RF' if cur == 'left' else 'LF'
    return make_contact, break_contact


def load_reference(conf):
    '''Load the reference trajectories (CoM, feet, CoP, phases of the gait) from conf.DATA_FILE_TSID.'''
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), conf.DATA_FILE_TSID)
    return dict(np.load(path))


def simulate_walking(conf, data=None, viewer=None, verbose=True):
    '''Simulate the walking of the biped with TSID. Returns a dictionary of logged quantities.'''
    if data is None:
        data = load_reference(conf)
    tsid_biped = TsidBiped(conf, viewer)

    contact_phase = data["contact_phase"]
    N = data["com"].shape[1]
    N_pre = int(conf.T_pre / conf.dt)
    N_post = int(conf.T_post / conf.dt)
    n_transition = int(round(conf.transition_time / conf.dt))
    make_contact, break_contact = compute_contact_events(contact_phase, n_transition)

    com_pos_ref = np.array(data["com"])
    com_vel_ref, com_acc_ref = data["dcom"], data["ddcom"]
    x_RF_ref, dx_RF_ref, ddx_RF_ref = np.array(data["x_RF"]), data["dx_RF"], data["ddx_RF"]
    x_LF_ref, dx_LF_ref, ddx_LF_ref = np.array(data["x_LF"]), data["dx_LF"], data["ddx_LF"]

    # express the reference trajectories w.r.t. the initial position of the right foot
    offset = tsid_biped.get_placement_RF().translation - x_RF_ref[:, 0]
    com_pos_ref += offset[:, None]
    x_RF_ref += offset[:, None]
    x_LF_ref += offset[:, None]
    cop_ref = np.array(data["cop"]) + offset[:2, None]

    log = dict(N=N, dt=conf.dt, contact_phase=contact_phase, com_ref=com_pos_ref, cop_ref=cop_ref, fail=None,
               com=np.full((3, N + N_post), nan),
               f_RF=np.zeros((6, N + N_post)), f_LF=np.zeros((6, N + N_post)),
               fMax_RF=np.full(N + N_post, nan), fMax_LF=np.full(N + N_post, nan),
               tau=np.zeros((tsid_biped.robot.na, N + N_post)),
               tau_max=tsid_biped.tau_max, joint_names=[tsid_biped.model.names[j + 2] for j in range(tsid_biped.robot.na)],
               hand=np.full((3, N + N_post), nan), hand_ref=np.full((3, N + N_post), nan),
               tray_tilt=np.full(N + N_post, nan))
    H_hand_0 = tsid_biped.H_hand_0
    q_list = []

    t = -conf.T_pre
    q, v = tsid_biped.q, tsid_biped.v
    if viewer:
        input("Wait for the robot to appear in the browser, then press enter to start")
    for i in range(-N_pre, N + N_post):
        time_start = time.time()

        if i in make_contact:
            if verbose:
                print("Time %.3f Adding contact %s" % (t, make_contact[i]))
            if make_contact[i] == 'LF':
                tsid_biped.add_contact_LF()
            else:
                tsid_biped.add_contact_RF()
        if i in break_contact:
            if verbose:
                print("Time %.3f Removing contact %s" % (t, break_contact[i]))
            if break_contact[i] == 'LF':
                tsid_biped.remove_contact_LF(conf.transition_time)
            else:
                tsid_biped.remove_contact_RF(conf.transition_time)

        if i < 0:
            tsid_biped.set_com_ref(com_pos_ref[:, 0], 0 * com_vel_ref[:, 0], 0 * com_acc_ref[:, 0])
        elif i < N:
            tsid_biped.set_com_ref(com_pos_ref[:, i], com_vel_ref[:, i], com_acc_ref[:, i])
            tsid_biped.set_LF_3d_ref(x_LF_ref[:, i], dx_LF_ref[:, i], ddx_LF_ref[:, i])
            tsid_biped.set_RF_3d_ref(x_RF_ref[:, i], dx_RF_ref[:, i], ddx_RF_ref[:, i])

        # reference of the tray (hand) task: keep the initial orientation and height of the hand,
        # and move it horizontally together with the reference of the CoM
        ### TODO ###
        # x-y: initial hand position + displacement of the CoM reference from its initial value;
        # z and orientation: the initial ones. Velocity/acceleration: those of the CoM reference
        # (x-y only), zero before and after the walk (the CoM reference is constant there).
        k = min(max(i, 0), N - 1)
        hand_pos_ref = H_hand_0.translation.copy()
        hand_pos_ref[:2] += com_pos_ref[:2, k] - com_pos_ref[:2, 0]
        hand_vel_ref, hand_acc_ref = np.zeros(3), np.zeros(3)
        if 0 <= i < N:
            hand_vel_ref[:2] = com_vel_ref[:2, i]
            hand_acc_ref[:2] = com_acc_ref[:2, i]
        tsid_biped.set_hand_ref(hand_pos_ref, H_hand_0.rotation, hand_vel_ref, hand_acc_ref)

        HQPData = tsid_biped.formulation.computeProblemData(t, q, v)
        sol = tsid_biped.solver.solve(HQPData)
        if sol.status != 0:
            print("Time %.3f QP problem could not be solved! Error code:" % t, sol.status)
            log["fail"] = t
            break
        if norm(v, 2) > 20.0:
            print("Time %.3f Velocities are too high, stop everything!" % t, norm(v))
            log["fail"] = t
            break

        # there is no ground in this simulation: if the robot loses its support it falls through
        # the floor, so stop as soon as the CoM drops well below its reference height
        com_z = tsid_biped.robot.com(tsid_biped.formulation.data())[2]
        if com_z < 0.8 * com_pos_ref[2, 0]:
            print("Time %.3f The robot is falling (CoM height %.3f m), stop everything!" % (t, com_z))
            log["fail"] = t
            break

        dv = tsid_biped.formulation.getAccelerations(sol)
        if i >= 0:
            log["com"][:, i] = tsid_biped.robot.com(tsid_biped.formulation.data())
            log["tau"][:, i] = tsid_biped.formulation.getActuatorForces(sol)
            log["fMax_RF"][i] = tsid_biped.contactRF.getMaxNormalForce
            log["fMax_LF"][i] = tsid_biped.contactLF.getMaxNormalForce
            H_hand = tsid_biped.get_placement_hand()
            log["hand"][:, i] = H_hand.translation
            log["hand_ref"][:, i] = hand_pos_ref
            # tilt of the tray = angle between its current and initial normal (vertical) axis
            R = H_hand.rotation @ H_hand_0.rotation.T
            log["tray_tilt"][i] = np.degrees(np.arccos(np.clip(R[2, 2], -1.0, 1.0)))
            # contact wrenches expressed in the foot frame: [fx, fy, fz, mx, my, mz]
            for contact, key in [(tsid_biped.contactRF, "f_RF"), (tsid_biped.contactLF, "f_LF")]:
                if tsid_biped.formulation.checkContact(contact.name, sol):
                    T = contact.getForceGeneratorMatrix
                    log[key][:, i] = T.dot(tsid_biped.formulation.getContactForce(contact.name, sol))

        if verbose and i % conf.PRINT_N == 0:
            print("Time %.3f" % t)
            print("\tnormal force RF: %.1f\tLF: %.1f" % (log["f_RF"][2, max(i, 0)], log["f_LF"][2, max(i, 0)]))
            print("\ttracking err %s: %.3f" % (tsid_biped.comTask.name.ljust(20, "."),
                                               norm(tsid_biped.comTask.position_error, 2)))

        q, v = tsid_biped.integrate_dv(q, v, dv, conf.dt)
        t += conf.dt

        if viewer and i % conf.DISPLAY_N == 0:
            q_list.append(q)
            tsid_biped.display(q)
            time_spent = time.time() - time_start
            if time_spent < conf.dt:
                time.sleep(conf.dt - time_spent)

    log["q_list"] = q_list
    log["tsid_biped"] = tsid_biped
    return log


def replay(log, conf):
    '''Play again in the viewer the motion of a simulation.'''
    for q in log["q_list"]:
        log["tsid_biped"].display(q)
        time.sleep(conf.DISPLAY_N * conf.dt)


def compute_metrics(log, conf):
    '''Scalar indicators of the quality of a walk.'''
    N, dt = log["N"], log["dt"]
    m = {}
    m["completed"] = log["fail"] is None and np.nanmin(log["com"][2, :N]) > 0.8 * log["com_ref"][2, 0]
    e = norm(log["com"][:2, :N] - log["com_ref"][:2, :N], axis=0)
    m["com_err_rms"] = np.sqrt(np.nanmean(e**2)) if m["completed"] else nan
    m["com_err_max"] = np.nanmax(e) if m["completed"] else nan
    fz = np.vstack([log["f_RF"][2, :N], log["f_LF"][2, :N]])
    m["max_force_rate"] = np.max(np.abs(np.diff(fz, axis=1))) / dt
    m["max_torque_rate"] = np.max(np.abs(np.diff(log["tau"][:, :N], axis=1))) / dt
    m["tray"] = conf.tray
    m["tray_tilt_max"] = np.nanmax(log["tray_tilt"][:N])
    m["hand_err_max"] = np.nanmax(norm(log["hand"][:, :N] - log["hand_ref"][:, :N], axis=0))
    m["hand_z_range"] = np.nanmax(log["hand"][2, :N]) - np.nanmin(log["hand"][2, :N])
    sat = np.max(np.abs(log["tau"][:, :N]), axis=1) / log["tau_max"]
    j = int(np.argmax(sat))
    m["max_torque_ratio"], m["max_torque_joint"] = sat[j], log["joint_names"][j]
    return m


def print_metrics(m):
    print("Walk completed:                   ", m["completed"])
    print("CoM tracking error RMS / max [m]:  %.4f / %.4f" % (m["com_err_rms"], m["com_err_max"]))
    print("Max normal force rate [N/s]:       %.0f" % m["max_force_rate"])
    print("Max joint torque rate [Nm/s]:      %.0f" % m["max_torque_rate"])
    if m["tray"]:
        print("Max tray tilt [deg]:               %.2f" % m["tray_tilt_max"])
        print("Max hand position error [m]:       %.4f" % m["hand_err_max"])
        print("Hand height range [m]:             %.4f" % m["hand_z_range"])
    print("Max |tau|/tau_max: %.2f (joint %s)" % (m["max_torque_ratio"], m["max_torque_joint"]))


def plot_walking(log, conf):
    N, dt = log["N"], log["dt"]
    time_ = np.arange(log["com"].shape[1]) * dt
    single = np.array([p != 'double' for p in log["contact_phase"]])

    def shade(ax):  # grey background = single support
        ax.fill_between(time_[:N], 0, 1, where=single[:N], color="grey", alpha=0.15,
                        transform=ax.get_xaxis_transform(), linewidth=0)

    (f, ax) = plut.create_empty_figure(2, 1)
    for i, lab in enumerate(["x", "y"]):
        ax[i].plot(time_, log["com"][i], label="CoM " + lab)
        ax[i].plot(time_[:N], log["com_ref"][i, :N], "r:", label="CoM ref " + lab)
        ax[i].plot(time_[:N], log["cop_ref"][i, :N], "g--", label="CoP ref " + lab)
        ax[i].set_ylabel("CoM, CoP [m]")
        shade(ax[i])
        ax[i].legend()
    ax[1].set_xlabel("Time [s]")

    (f, ax) = plut.create_empty_figure(2, 1)
    ax[0].plot(time_, log["f_RF"][2], label="RF normal force")
    ax[0].plot(time_, log["f_LF"][2], label="LF normal force")
    ax[0].set_ylabel("Force [N]")
    ax[1].plot(time_, np.minimum(log["fMax_RF"], 1e3), label="RF max normal force")
    ax[1].plot(time_, np.minimum(log["fMax_LF"], 1e3), "--", label="LF max normal force")
    ax[1].set_ylabel("Force [N] (clipped at 1e3)")
    for a in ax:
        shade(a)
        a.legend()
    ax[1].set_xlabel("Time [s]")

    if not conf.tray:
        plot_torques(log, time_, shade)
        return

    (f, ax) = plut.create_empty_figure(4, 1)
    for i, lab in enumerate(["x", "y", "z"]):
        ax[i].plot(time_, log["hand"][i], label="hand " + lab)
        ax[i].plot(time_, log["hand_ref"][i], "r:", label="hand ref " + lab)
        ax[i].set_ylabel("[m]")
    ax[3].plot(time_, log["tray_tilt"], label="tray tilt")
    ax[3].set_ylabel("[deg]")
    for a in ax:
        shade(a)
        a.legend()
    ax[3].set_xlabel("Time [s]")

    plot_torques(log, time_, shade)


def plot_torques(log, time_, shade):
    (f, ax) = plut.create_empty_figure(1, 1)
    for j in range(log["tau"].shape[0]):
        tau_normalized = log["tau"][j] / log["tau_max"][j]
        # plot torques only for joints that reached 50% of max torque
        if np.max(np.abs(tau_normalized)) > 0.5:
            ax.plot(time_, tau_normalized, alpha=0.7, label=log["joint_names"][j])
    ax.axhline(1.0, color="k", linestyle=":")
    ax.axhline(-1.0, color="k", linestyle=":")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Normalized torque")
    shade(ax)
    ax.legend()


if __name__ == "__main__":
    conf = romeo_conf
    print("".center(conf.LINE_WIDTH, "#"))
    print(" Biped walking with double support ".center(conf.LINE_WIDTH, "#"))
    print("".center(conf.LINE_WIDTH, "#"), "\n")

    log = simulate_walking(conf, viewer=conf.viewer)
    print_metrics(compute_metrics(log, conf))

    plot_walking(log, conf)
    if conf.viewer:
        plt.show(block=False)  # show the plots now, while the replay of the motion is offered
        plt.pause(0.5)
        while (input("Play video again? [Y]/n: ") or "Y").lower() == "y":
            replay(log, conf)
    plt.show()
