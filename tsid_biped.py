import os
import subprocess
import time

import numpy as np
import pinocchio as pin
import tsid


class TsidBiped:
    ''' Standard TSID formulation for a biped robot standing on its rectangular feet.
        - Center of mass task
        - Postural task
        - 6d rigid contact constraint for both feet
        - Regularization task for contact forces
    '''

    def __init__(self, conf, viewer=None):
        self.conf = conf
        # build the model and attach the payload (the tray) to the hand: the payload is modelled
        # as a point-like mass rigidly attached to the link of the joint conf.payload_joint
        pin_model = pin.buildModelFromUrdf(conf.urdf, pin.JointModelFreeFlyer())
        if conf.tray and conf.payload_mass > 0.0:
            jid = pin_model.getJointId(conf.payload_joint)
            payload = pin.Inertia.FromSphere(conf.payload_mass, 0.02)
            pin_model.inertias[jid] += payload.se3Action(pin.SE3(np.eye(3), np.array(conf.payload_offset)))
        self.robot = tsid.RobotWrapper(pin_model, tsid.FLOATING_BASE_SYSTEM, False)
        robot = self.robot
        self.model = robot.model()
        pin.loadReferenceConfigurations(self.model, conf.srdf, False)
        self.q0 = q = self.model.referenceConfigurations["half_sitting"]
        v = np.zeros(robot.nv)

        assert self.model.existFrame(conf.rf_frame_name)
        assert self.model.existFrame(conf.lf_frame_name)

        formulation = tsid.InverseDynamicsFormulationAccForce("tsid", robot, False)
        formulation.computeProblemData(0.0, q, v)
        data = formulation.data()
        contact_Point = np.ones((3, 4)) * (-conf.lz)
        contact_Point[0, :] = [-conf.lxn, -conf.lxn, conf.lxp, conf.lxp]
        contact_Point[1, :] = [-conf.lyn, conf.lyp, -conf.lyn, conf.lyp]

        contactRF = tsid.Contact6d("contact_rfoot", robot, conf.rf_frame_name, contact_Point,
                                   conf.contactNormal, conf.mu, conf.fMin, conf.fMax)
        contactRF.setKp(conf.kp_contact * np.ones(6))
        contactRF.setKd(2.0 * np.sqrt(conf.kp_contact) * np.ones(6))
        self.RF = robot.model().getFrameId(conf.rf_frame_name)
        H_rf_ref = robot.framePosition(data, self.RF)

        # modify initial robot configuration so that foot is on the ground (z=0)
        q[2] -= H_rf_ref.translation[2] - conf.lz
        formulation.computeProblemData(0.0, q, v)
        data = formulation.data()
        H_rf_ref = robot.framePosition(data, self.RF)
        contactRF.setReference(H_rf_ref)
        if conf.w_contact >= 0.0:
            formulation.addRigidContact(contactRF, conf.w_forceRef, conf.w_contact, 1)
        else:
            formulation.addRigidContact(contactRF, conf.w_forceRef)

        contactLF = tsid.Contact6d("contact_lfoot", robot, conf.lf_frame_name, contact_Point,
                                   conf.contactNormal, conf.mu, conf.fMin, conf.fMax)
        contactLF.setKp(conf.kp_contact * np.ones(6))
        contactLF.setKd(2.0 * np.sqrt(conf.kp_contact) * np.ones(6))
        self.LF = robot.model().getFrameId(conf.lf_frame_name)
        H_lf_ref = robot.framePosition(data, self.LF)
        contactLF.setReference(H_lf_ref)
        if conf.w_contact >= 0.0:
            formulation.addRigidContact(contactLF, conf.w_forceRef, conf.w_contact, 1)
        else:
            formulation.addRigidContact(contactLF, conf.w_forceRef)

        comTask = tsid.TaskComEquality("task-com", robot)
        comTask.setKp(conf.kp_com * np.ones(3))
        comTask.setKd(2.0 * np.sqrt(conf.kp_com) * np.ones(3))
        formulation.addMotionTask(comTask, conf.w_com, 1, 0.0)
        
        copTask = tsid.TaskCopEquality("task-cop", robot)
        formulation.addForceTask(copTask, conf.w_cop, 1, 0.0)
        
        amTask = tsid.TaskAMEquality("task-am", robot)
        amTask.setKp(conf.kp_am * np.array([1., 1., 0.]))
        amTask.setKd(2.0 * np.sqrt(conf.kp_am * np.array([1., 1., 0.])))
        formulation.addMotionTask(amTask, conf.w_am, 1, 0.)
        sampleAM = tsid.TrajectorySample(3)
        amTask.setReference(sampleAM)


        postureTask = tsid.TaskJointPosture("task-posture", robot)
        postureTask.setKp(conf.kp_posture * conf.gain_vector)
        postureTask.setKd(2.0 * np.sqrt(conf.kp_posture * conf.gain_vector))
        postureTask.setMask(conf.masks_posture)
        formulation.addMotionTask(postureTask, conf.w_posture, 1, 0.0)

        self.leftFootTask = tsid.TaskSE3Equality("task-left-foot", self.robot, self.conf.lf_frame_name)
        self.leftFootTask.setKp(self.conf.kp_foot * np.ones(6))
        self.leftFootTask.setKd(2.0 * np.sqrt(self.conf.kp_foot) * np.ones(6))
        self.trajLF = tsid.TrajectorySE3Constant("traj-left-foot", H_lf_ref)
        formulation.addMotionTask(self.leftFootTask, self.conf.w_foot, 1, 0.0)

        self.rightFootTask = tsid.TaskSE3Equality("task-right-foot", self.robot, self.conf.rf_frame_name)
        self.rightFootTask.setKp(self.conf.kp_foot * np.ones(6))
        self.rightFootTask.setKd(2.0 * np.sqrt(self.conf.kp_foot) * np.ones(6))
        self.trajRF = tsid.TrajectorySE3Constant("traj-right-foot", H_rf_ref)
        formulation.addMotionTask(self.rightFootTask, self.conf.w_foot, 1, 0.0)

        self.tau_max = conf.tau_max_scaling*robot.model().effortLimit[-robot.na:]
        self.tau_min = -self.tau_max
        actuationBoundsTask = tsid.TaskActuationBounds("task-actuation-bounds", robot)
        actuationBoundsTask.setBounds(self.tau_min, self.tau_max)
        if conf.w_torque_bounds > 0.0:
            formulation.addActuationTask(actuationBoundsTask, conf.w_torque_bounds, 0, 0.0)

        jointBoundsTask = tsid.TaskJointBounds("task-joint-bounds", robot, conf.dt)
        self.v_max = conf.v_max_scaling * robot.model().velocityLimit[-robot.na:]
        self.v_min = -self.v_max
        jointBoundsTask.setVelocityBounds(self.v_min, self.v_max)
        if conf.w_joint_bounds > 0.0:
            formulation.addMotionTask(jointBoundsTask, conf.w_joint_bounds, 0, 0.0)

        com_ref = robot.com(data)
        self.trajCom = tsid.TrajectoryEuclidianConstant("traj_com", com_ref)
        self.sample_com = self.trajCom.computeNext()

        q_ref = q[7:]
        self.trajPosture = tsid.TrajectoryEuclidianConstant("traj_joint", q_ref)
        postureTask.setReference(self.trajPosture.computeNext())

        self.sampleLF = self.trajLF.computeNext()
        self.sample_LF_pos = self.sampleLF.value()
        self.sample_LF_vel = self.sampleLF.derivative()
        self.sample_LF_acc = self.sampleLF.second_derivative()

        self.sampleRF = self.trajRF.computeNext()
        self.sample_RF_pos = self.sampleRF.value()
        self.sample_RF_vel = self.sampleRF.derivative()
        self.sample_RF_acc = self.sampleRF.second_derivative()

        # tray task: SE3 task on the hand carrying the tray
        self.handTask = tsid.TaskSE3Equality("task-hand", self.robot, conf.hand_frame_name)
        ### TODO ###
        # set the gains of the hand task (kp = conf.kp_hand, kd = 2 sqrt(kp)), its mask (conf.hand_mask),
        # the frame in which it is expressed (conf.hand_local_frame, see TaskSE3Equality.useLocalFrame)
        # and add it to the formulation with weight conf.w_hand and priority level conf.hand_priority
        # (only if conf.w_hand > 0 or the priority level is 0)

        self.HAND = self.model.getFrameId(conf.hand_frame_name)
        self.H_hand_0 = robot.framePosition(data, self.HAND).copy()  # initial pose of the hand
        self.sampleHand = tsid.TrajectorySample(12, 6)

        self.solver = tsid.SolverHQuadProgFast("qp solver")
        self.solver.resize(formulation.nVar, formulation.nEq, formulation.nIn)

        self.comTask = comTask
        self.copTask = copTask
        self.amTask = amTask
        self.postureTask = postureTask
        self.contactRF = contactRF
        self.contactLF = contactLF
        self.actuationBoundsTask = actuationBoundsTask
        self.jointBoundsTask = jointBoundsTask
        self.formulation = formulation
        self.q = q
        self.v = v

        self.contact_LF_active = True
        self.contact_RF_active = True

        if viewer:
            self.robot_display = pin.RobotWrapper.BuildFromURDF(conf.urdf, [conf.path], pin.JointModelFreeFlyer())
            if viewer.__name__ == 'GepettoVisualizer':
                import gepetto.corbaserver
                launched = subprocess.getstatusoutput("ps aux |grep 'gepetto-gui'|grep -v 'grep'|wc -l")
                if int(launched[1]) == 0:
                    os.system('gepetto-gui &')
                time.sleep(1)
                self.viz = viewer(self.robot_display.model, self.robot_display.collision_model,
                                  self.robot_display.visual_model)
                self.viz.initViewer(loadModel=True)
                self.viz.displayCollisions(False)
                self.viz.displayVisuals(True)
                self.viz.display(q)

                self.gui = self.viz.viewer.gui
                # self.gui.setCameraTransform(0, conf.CAMERA_TRANSFORM)
                self.gui.addFloor('world/floor')
                self.gui.setLightingMode('world/floor', 'OFF')
            elif viewer.__name__ == 'MeshcatVisualizer':
                self.viz = viewer(self.robot_display.model, self.robot_display.collision_model,
                                  self.robot_display.visual_model)
                self.viz.initViewer(loadModel=True, open=True)
                if conf.tray:
                    self.add_tray_to_viewer(q)
                self.display(q)

    def add_tray_to_viewer(self, q):
        ''' Draw a tray with two glasses, rigidly attached to the hand (meshcat only). '''
        import meshcat.geometry as g
        model, data = self.robot_display.model, self.robot_display.data
        self.tray_frame_id = model.getFrameId(self.conf.hand_frame_name)
        pin.framesForwardKinematics(model, data, q)
        self.H_hand_display_0 = data.oMf[self.tray_frame_id].copy()
        # the tray is horizontal, with its top surface at the position of the payload
        top = self.H_hand_display_0.act(np.array(self.conf.payload_offset))
        thickness, glass_h, glass_r = 0.01, 0.08, 0.025
        rot_cyl = pin.utils.rotate('x', np.pi / 2)  # meshcat cylinders are aligned with the y axis
        self.tray_objects = {
            "tray/board": (g.Box([0.22, 0.16, thickness]), 0x8b5a2b,
                           pin.SE3(np.eye(3), top - np.array([0, 0, thickness / 2]))),
            "tray/glass_1": (g.Cylinder(glass_h, glass_r), 0x66ccff,
                             pin.SE3(rot_cyl, top + np.array([0.05, 0.03, glass_h / 2]))),
            "tray/glass_2": (g.Cylinder(glass_h, glass_r), 0x66ccff,
                             pin.SE3(rot_cyl, top + np.array([-0.05, -0.03, glass_h / 2]))),
        }
        for name, (geometry, color, _) in self.tray_objects.items():
            material = g.MeshLambertMaterial(color=color, opacity=0.8 if 'glass' in name else 1.0,
                                             transparent='glass' in name)
            self.viz.viewer[name].set_object(geometry, material)

    def display(self, q):
        if hasattr(self, 'viz'):
            self.viz.display(q)
            if hasattr(self, 'tray_objects'):
                model, data = self.robot_display.model, self.robot_display.data
                pin.framesForwardKinematics(model, data, q)
                # move the tray with the hand: T(t) = H_hand(t) * H_hand(0)^-1 * T(0)
                H = data.oMf[self.tray_frame_id] * self.H_hand_display_0.inverse()
                for name, (_, _, T0) in self.tray_objects.items():
                    self.viz.viewer[name].set_transform((H * T0).homogeneous)

    def integrate_dv(self, q, v, dv, dt):
        v_mean = v + 0.5 * dt * dv
        v += dt * dv
        q = pin.integrate(self.model, q, dt * v_mean)
        return q, v

    def get_placement_hand(self):
        return self.robot.framePosition(self.formulation.data(), self.HAND)

    def set_hand_ref(self, pos, rot, vel, acc):
        ''' Set the reference of the hand task: position pos, orientation rot (3x3 rotation matrix),
            linear velocity vel and linear acceleration acc (angular velocity/acceleration = 0). '''
        self.sampleHand.pos(np.concatenate([pos, rot.flatten('F')]))
        self.sampleHand.vel(np.concatenate([vel, np.zeros(3)]))
        self.sampleHand.acc(np.concatenate([acc, np.zeros(3)]))
        self.handTask.setReference(self.sampleHand)

    def get_placement_LF(self):
        return self.robot.framePosition(self.formulation.data(), self.LF)

    def get_placement_RF(self):
        return self.robot.framePosition(self.formulation.data(), self.RF)

    def set_com_ref(self, pos, vel, acc):
        self.sample_com.value(pos)
        self.sample_com.derivative(vel)
        self.sample_com.second_derivative(acc)
        self.comTask.setReference(self.sample_com)

    def set_RF_3d_ref(self, pos, vel, acc):
        self.sample_RF_pos[:3] = pos
        self.sample_RF_vel[:3] = vel
        self.sample_RF_acc[:3] = acc
        self.sampleRF.pos(self.sample_RF_pos)
        self.sampleRF.vel(self.sample_RF_vel)
        self.sampleRF.acc(self.sample_RF_acc)
        self.rightFootTask.setReference(self.sampleRF)

    def set_LF_3d_ref(self, pos, vel, acc):
        self.sample_LF_pos[:3] = pos
        self.sample_LF_vel[:3] = vel
        self.sample_LF_acc[:3] = acc
        self.sampleLF.pos(self.sample_LF_pos)
        self.sampleLF.vel(self.sample_LF_vel)
        self.sampleLF.acc(self.sample_LF_acc)
        self.leftFootTask.setReference(self.sampleLF)

    def get_LF_3d_pos_vel_acc(self, dv):
        data = self.formulation.data()
        H = self.robot.framePosition(data, self.LF)
        v = self.robot.frameVelocity(data, self.LF)
        a = self.leftFootTask.getAcceleration(dv)
        return H.translation, v.linear, a[:3]

    def get_RF_3d_pos_vel_acc(self, dv):
        data = self.formulation.data()
        H = self.robot.framePosition(data, self.RF)
        v = self.robot.frameVelocity(data, self.RF)
        a = self.rightFootTask.getAcceleration(dv)
        return H.translation, v.linear, a[:3]

    def remove_contact_RF(self, transition_time=0.0):
        ''' Remove the contact of the right foot. If transition_time > 0, TSID keeps the
            contact active for transition_time seconds, while linearly decreasing the maximum
            normal force from its current value to zero, and only then removes it. '''
        H_rf_ref = self.robot.framePosition(self.formulation.data(), self.RF)
        self.trajRF.setReference(H_rf_ref)
        self.rightFootTask.setReference(self.trajRF.computeNext())

        self.formulation.removeRigidContact(self.contactRF.name, transition_time)
        self.contact_RF_active = False

    def remove_contact_LF(self, transition_time=0.0):
        ''' Remove the contact of the left foot. If transition_time > 0, TSID keeps the
            contact active for transition_time seconds, while linearly decreasing the maximum
            normal force from its current value to zero, and only then removes it. '''
        H_lf_ref = self.robot.framePosition(self.formulation.data(), self.LF)
        self.trajLF.setReference(H_lf_ref)
        self.leftFootTask.setReference(self.trajLF.computeNext())

        self.formulation.removeRigidContact(self.contactLF.name, transition_time)
        self.contact_LF_active = False

    def add_contact_RF(self, transition_time=0.0):
        H_rf_ref = self.robot.framePosition(self.formulation.data(), self.RF)
        self.contactRF.setReference(H_rf_ref)
        # Q1 fix: removeRigidContact(name, transition_time > 0) ramps the max normal force of this
        # Contact6d down to 0 and leaves it there; restore the nominal value before re-adding it
        self.contactRF.setMaxNormalForce(self.conf.fMax)
        if self.conf.w_contact >= 0.0:
            self.formulation.addRigidContact(self.contactRF, self.conf.w_forceRef, self.conf.w_contact, 1)
        else:
            self.formulation.addRigidContact(self.contactRF, self.conf.w_forceRef)

        self.contact_RF_active = True

    def add_contact_LF(self, transition_time=0.0):
        H_lf_ref = self.robot.framePosition(self.formulation.data(), self.LF)
        self.contactLF.setReference(H_lf_ref)
        # Q1 fix: restore the nominal max normal force (ramped to 0 by remove_contact_LF)
        self.contactLF.setMaxNormalForce(self.conf.fMax)
        if self.conf.w_contact >= 0.0:
            self.formulation.addRigidContact(self.contactLF, self.conf.w_forceRef, self.conf.w_contact, 1)
        else:
            self.formulation.addRigidContact(self.contactLF, self.conf.w_forceRef)

        self.contact_LF_active = True
