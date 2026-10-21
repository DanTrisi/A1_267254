"""
Compare the walk with an empty tray and with a heavy tray (question 5).

Two walks are simulated with the tray task (w_hand = 100, transition_time = 0.1) and played
together in the same viewer:
  - solid robot  : heavy tray (PAYLOADS["solid"], 5 kg). Romeo cannot carry it with its nominal
                   torque limits, so they are raised for this walk (TAU_MAX_SCALING["solid"])
  - blue ghost   : baseline walk, with the empty tray (0.5 kg) and the default parameters
  - red markers  : total CoM (robot + tray) and its projection on the ground
  - blue markers : projection of the pelvis on the ground
At the end, the lateral positions of the pelvis and of the total CoM are plotted for both walks.
The tray task and the fix of question 1 must be implemented before running this script.

    python compare_payloads.py
"""
import importlib
import time

import matplotlib.pyplot as plt
import meshcat
import meshcat.geometry as g
import meshcat.transformations as tf
import numpy as np
import pinocchio as pin
import pinocchio.visualize as pv
import romeo_conf
import biped_walking
from tsid_biped import TsidBiped

PAYLOADS = {"solid": 5.0, "ghost": 0.5}  # [kg]
TAU_MAX_SCALING = {"solid": 4.0, "ghost": None}  # torque limits (x nominal); None = default of romeo_conf
GHOST_COLOR = 0x3399ff     # colour of the ghost robot
GHOST_OPACITY = 0.35
DISPLAY_EVERY = 10                       # display one control step every DISPLAY_EVERY


class RecordingTsidBiped(TsidBiped):
    '''TsidBiped that records the configuration at every control step.'''
    recorded = []

    def integrate_dv(self, q, v, dv, dt):
        q, v = super().integrate_dv(q, v, dv, dt)
        RecordingTsidBiped.recorded[-1].append(q.copy())
        return q, v


def simulate(payload, tau_max_scaling=None):
    conf = importlib.reload(romeo_conf)
    conf.viewer = None
    conf.tray = True
    conf.transition_time = 0.1
    conf.w_hand = 100.0
    conf.payload_mass = payload
    if tau_max_scaling is not None:
        conf.tau_max_scaling = tau_max_scaling
    RecordingTsidBiped.recorded.append([])
    biped_walking.TsidBiped = RecordingTsidBiped
    log = biped_walking.simulate_walking(conf, viewer=None, verbose=False)
    biped_walking.TsidBiped = TsidBiped
    if log["fail"] is not None:
        raise SystemExit("The walk with a %.1f kg tray failed: check your tray task and the fix of question 1"
                         % payload)
    Q = np.array(RecordingTsidBiped.recorded[-1])
    model = log["tsid_biped"].robot.model()  # model including the payload, for the total CoM
    data = model.createData()
    com = np.array([pin.centerOfMass(model, data, q) for q in Q])
    return conf, Q, com


def add_tray(vis, conf, prefix):
    thickness, glass_h, glass_r = 0.01, 0.08, 0.025
    objects = {"board": (g.Box([0.22, 0.16, thickness]), 0x8b5a2b, np.array([0, 0, -thickness / 2]), False),
               "glass_1": (g.Cylinder(glass_h, glass_r), 0x66ccff, np.array([0.05, 0.03, glass_h / 2]), True),
               "glass_2": (g.Cylinder(glass_h, glass_r), 0x66ccff, np.array([-0.05, -0.03, glass_h / 2]), True)}
    for name, (geom, color, _, is_glass) in objects.items():
        vis[prefix + "/" + name].set_object(geom, g.MeshLambertMaterial(color=color, opacity=0.8 if is_glass else 1.0,
                                                                        transparent=is_glass))
    return objects


class GhostRobot:
    '''Robot drawn with a single translucent colour. The .dae meshes of Romeo carry their own
       materials, which meshcat cannot recolour, so the ghost is drawn from the triangle meshes
       that pinocchio loads for the visual model.'''

    def __init__(self, vis, robot, root, color, opacity):
        self.vis, self.root = vis, root
        self.model, self.gmodel = robot.model, robot.visual_model
        self.data, self.gdata = self.model.createData(), self.gmodel.createData()
        material = g.MeshLambertMaterial(color=color, opacity=opacity, transparent=True)
        for go in self.gmodel.geometryObjects:
            vis[root + "/" + go.name].set_object(pv.meshcat_visualizer.loadMesh(go.geometry), material)

    def display(self, q):
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateGeometryPlacements(self.model, self.data, self.gmodel, self.gdata)
        for go, M in zip(self.gmodel.geometryObjects, self.gdata.oMg):
            self.vis[self.root + "/" + go.name].set_transform(M.homogeneous)


def marker(vis, name, color, radius, opacity=1.0):
    vis[name].set_object(g.Sphere(radius), g.MeshLambertMaterial(color=color, opacity=opacity, transparent=opacity < 1))


if __name__ == "__main__":
    print("Simulating the two walks...")
    runs = {k: simulate(m, TAU_MAX_SCALING[k]) for k, m in PAYLOADS.items()}
    conf = runs["solid"][0]
    n = min(len(runs["solid"][1]), len(runs["ghost"][1]))

    # ---------------- viewer: two robots in the same scene ----------------
    vis = meshcat.Visualizer()
    vis.open()
    vis.delete()
    viz = {}
    robot = pin.RobotWrapper.BuildFromURDF(conf.urdf, [conf.path], pin.JointModelFreeFlyer())
    viz["solid"] = pv.MeshcatVisualizer(robot.model, robot.collision_model, robot.visual_model)
    viz["solid"].initViewer(viewer=vis)
    viz["solid"].loadViewerModel(rootNodeName="robot_solid")
    ghost_robot = pin.RobotWrapper.BuildFromURDF(conf.urdf, [conf.path], pin.JointModelFreeFlyer())
    viz["ghost"] = GhostRobot(vis, ghost_robot, "robot_ghost", GHOST_COLOR, GHOST_OPACITY)
    tray = add_tray(vis, conf, "tray")
    marker(vis, "com_solid", 0xff0000, 0.03)
    marker(vis, "com_solid_ground", 0xff0000, 0.02)
    marker(vis, "pelvis_solid_ground", 0x0000ff, 0.02)
    marker(vis, "com_ghost", 0xff8888, 0.03, 0.5)
    marker(vis, "com_ghost_ground", 0xff8888, 0.02, 0.5)
    marker(vis, "pelvis_ghost_ground", 0x8888ff, 0.02, 0.5)

    # tray placement w.r.t. the hand, computed in the initial configuration
    model_d = robot.model
    data_d = model_d.createData()
    hand_id = model_d.getFrameId(conf.hand_frame_name)
    pin.framesForwardKinematics(model_d, data_d, runs["solid"][1][0])
    H_hand_0 = data_d.oMf[hand_id].copy()
    top_0 = H_hand_0.act(np.array(conf.payload_offset))

    def show(i):
        for k in ["solid", "ghost"]:
            _, Q, com = runs[k]
            q = Q[i]
            viz[k].display(q)
            vis["com_" + k].set_transform(tf.translation_matrix(com[i]))
            vis["com_%s_ground" % k].set_transform(tf.translation_matrix([com[i][0], com[i][1], 0.0]))
            vis["pelvis_%s_ground" % k].set_transform(tf.translation_matrix([q[0], q[1], 0.0]))
        pin.framesForwardKinematics(model_d, data_d, runs["solid"][1][i])
        H = data_d.oMf[hand_id] * H_hand_0.inverse()
        rot_cyl = pin.utils.rotate('x', np.pi / 2)
        for name, (_, _, offset, is_glass) in tray.items():
            T0 = pin.SE3(rot_cyl if is_glass else np.eye(3), top_0 + offset)
            vis["tray/" + name].set_transform((H * T0).homogeneous)
        # camera in front of the robot, following it
        x = runs["solid"][1][i][0]
        vis["/Cameras/default"].set_transform(tf.translation_matrix([x, 0.0, 0.45]))

    vis["/Cameras/default/rotated/<object>"].set_property("position", [2.2, 0.35, 0.0])  # in front, slightly above
    show(0)
    print(__doc__)
    print("Viewer:", vis.url())
    try:
        input("Wait for the robots to appear in the browser, then press enter to start")
    except EOFError:
        time.sleep(8)

    def play():
        for i in range(0, n, DISPLAY_EVERY):
            t0 = time.time()
            show(i)
            dt = DISPLAY_EVERY * conf.dt - (time.time() - t0)
            if dt > 0:
                time.sleep(dt)

    play()

    # ---------------- plot ----------------
    t = np.arange(n) * conf.dt - conf.T_pre
    fig, ax = plt.subplots(2, 1, sharex=True)
    for k, style in [("solid", "-"), ("ghost", "--")]:
        _, Q, com = runs[k]
        lab = "%.1f kg tray" % PAYLOADS[k]
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
    plt.show(block=False)
    plt.pause(0.5)
    try:
        while (input("Play again? [Y]/n: ") or "Y").lower() == "y":
            play()
    except EOFError:
        pass
    plt.show()
