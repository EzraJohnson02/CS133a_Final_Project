import numpy as np
from math import pi, sin, cos, acos, atan2, sqrt, fmod, exp
from finalproject.TransformHelpers     import *
from finalproject.TrajectoryPlanner    import *

# Grab the utilities
from finalproject.TransformHelpers   import *
from finalproject.TrajectoryUtils    import *
from scipy.optimize import fmin

from finalproject.KinematicChain     import KinematicChain
from std_msgs.msg import Float64

class Trajectory:
    # Initialization.
    def __init__(self, node, p_ball, v_ball, a_ball):
        # Set up the kinematic chain object.
        self.chain = KinematicChain(node, 'world', 'ee_link', self.jointnames())

        # Define the various points.
        self.q0 = np.radians(np.array([0,0,-np.pi,0,0,0]).reshape((-1,1)))
        self.p0 = np.zeros(3).reshape((-1,1))
        self.R0 = Reye()

        min_time = 0.5
        tp = TrajectoryPlanner(p_ball, v_ball, a_ball, min_time)

        t = tp.calculate_min_time()
        catch_position = tp.position(t)

        distance = np.linalg.norm(catch_position)
        task_radius = 1.8
        if distance > task_radius:
            print(f"Ball 1 is out of bounds! Distance {distance}")
            self.pF = self.p0
            self.T = 10
            e = ez()
            self.e_paddle = e
            self.theta_paddle = 0
        else:
            self.pF = np.array(catch_position).reshape((-1,1))
            self.T = t * 0.9
            ball_velocity = tp.velocity(t).flatten()
            # Negative to go in other direction
            ball_velocity_norm = -ball_velocity / np.linalg.norm(ball_velocity)
            z = ez().flatten()
            e = np.cross(ball_velocity_norm, z).reshape((3,1))
            e = e / np.linalg.norm(e)

            self.e_paddle = e
            self.theta_paddle = np.arccos(np.dot(ball_velocity_norm, z))


        # tp = TrajectoryPlanner(p_ball2, v_ball2, a_ball2, min_time)

        # t = tp.calculate_min_time()
        # catch_position = tp.position(t)

        # distance = np.linalg.norm(catch_position)
        # task_radius = 1.8
        # if distance > task_radius:
        #     print(f"Ball 2 is out of bounds! Distance {distance}")
        #     self.pF2 = self.p0
        #     self.T2 = 10
        #     e = ez()
        #     self.e_paddle2 = e
        #     self.theta_paddle2 = 0
        # else:
        #     self.pF2 = np.array(catch_position).reshape((-1,1))
        #     self.T2 = t * 0.9
        #     ball_velocity = tp.velocity(t).flatten()
        #     # Negative to go in other direction
        #     ball_velocity_norm = -ball_velocity / np.linalg.norm(ball_velocity)
        #     z = ez().flatten()
        #     e = np.cross(ball_velocity_norm, z).reshape((3,1))
        #     e = e / np.linalg.norm(e)

        #     self.e_paddle2 = e
        #     self.theta_paddle2 = np.arccos(np.dot(ball_velocity_norm, z))
        self.q = self.q0
        # self.q2 = self.q0
        self.pd = self.p0
        # self.pd2 = self.p0
        self.Rd = self.R0 @ Roty(-pi/2) @ Rote(self.e_paddle, -self.theta_paddle)
        # self.Rd2 = self.R0 @ Roty(-pi/2) @ Rote(self.e_paddle2, -self.theta_paddle2)

        self.lam = 20

        self.pub = node.create_publisher(Float64, '/condition', 10)

    def set_goal(self, p_ball, v_ball, a_ball):
        min_time = 0.2
        tp = TrajectoryPlanner(p_ball, v_ball, a_ball, min_time)

        t = tp.calculate_min_time()
        catch_position = tp.position(t)

        distance = np.linalg.norm(catch_position)
        task_radius = 1.8
        if distance > task_radius:
            print(f"Ball 1 is out of bounds! Distance {distance}")
            self.pF = self.p0
            self.T = 10
            e = ez()
            self.e_paddle = e
            self.theta_paddle = 0
        else:
            self.pF = np.array(catch_position).reshape((-1,1))
            self.T = t * 0.9
            ball_velocity = tp.velocity(t).flatten()
            # Negative to go in other direction
            ball_velocity_norm = -ball_velocity / np.linalg.norm(ball_velocity)
            z = ez().flatten()
            e = np.cross(ball_velocity_norm, z).reshape((3,1))
            e = e / np.linalg.norm(e)

            self.e_paddle = e
            self.theta_paddle = np.arccos(np.dot(ball_velocity_norm, z))


        # tp = TrajectoryPlanner(p_ball2, v_ball2, a_ball2, min_time)

        # t = tp.calculate_min_time()
        # catch_position = tp.position(t)

        # distance = np.linalg.norm(catch_position)
        # task_radius = 1.8
        # if distance > task_radius:
        #     print(f"Ball 2 is out of bounds! Distance {distance}")
        #     self.pF2 = self.p0
        #     self.T2 = 10
        #     e = ez()
        #     self.e_paddle2 = e
        #     self.theta_paddle2 = 0
        # else:
        #     self.pF2 = np.array(catch_position).reshape((-1,1))
        #     self.T2 = t * 0.9
        #     ball_velocity = tp.velocity(t).flatten()
        #     # Negative to go in other direction
        #     ball_velocity_norm = -ball_velocity / np.linalg.norm(ball_velocity)
        #     z = ez().flatten()
        #     e = np.cross(ball_velocity_norm, z).reshape((3,1))
        #     e = e / np.linalg.norm(e)

        #     self.e_paddle2 = e
        #     self.theta_paddle2 = np.arccos(np.dot(ball_velocity_norm, z))

        # self.Rd2 = self.R0 @ Roty(-pi/2) @ Rote(e, -self.theta_paddle2)
        self.Rd = self.R0 @ Roty(-pi/2) @ Rote(e, -self.theta_paddle)


    # Declare the joint names.
    def jointnames(self):
        # Return a list of joint names FOR THE EXPECTED URDF!
        return [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]

    def position(self):
        (p, _, _, _) = self.chain.fkin(self.q)
        return p

    # Evaluate at the given time.  This was last called (dt) ago.
    def evaluate(self, t, dt):
        if t > self.T:
            return None
        (s0, s0dot) = goto(t, self.T, 0.0, 1.0)
        pd = self.p0 + (self.pF - self.p0) * s0
        vd = (self.pF - self.p0) * s0dot
        wd = -pi/2 * ey() - self.theta_paddle * self.e_paddle
        # pd2 = self.p0 + (self.pF2 - self.p0) * s0
        # vd2 = (self.pF2 - self.p0) * s0dot
        # wd2 = -pi/2 * ey() - self.theta_paddle2 * self.e_paddle2

        qlast = self.q
        pdlast = self.pd
        Rdlast = self.Rd

        # qlast2 = self.q2
        # pdlast2 = self.pd2
        # Rdlast2 = self.Rd2

        (p, R, Jv, Jw) = self.chain.fkin(qlast)
        vr = vd + self.lam * ep(pdlast, p)
        wr = wd + self.lam * eR(Rdlast, R)
        J = np.vstack((Jv, Jw))
        xrdot = np.vstack((vr, wr))

        # (p2, R2, Jv2, Jw2) = self.chain.fkin(qlast2)
        # vr2 = vd2 + self.lam * ep(pdlast2, p2)
        # wr2 = wd2 + self.lam * eR(Rdlast2, R2)
        # J2 = np.vstack((Jv2, Jw2))
        # xrdot2 = np.vstack((vr2, wr2))

        M, N = J.shape
        gamma = 0.1
        lam_s = 10
        Jp = np.transpose(J) @ np.linalg.pinv(J @ np.transpose(J) + gamma ** 2 * np.eye(M))
        qdot = Jp @ xrdot

        q = qlast + dt * qdot
        # q2 = qlast2 + dt * qdot2

        self.q = q
        # self.q2 = q2

        self.pd = pd
        # self.pd2 = pd2

        # Return the position and velocity as python lists.
        return (q.flatten().tolist(), qdot.flatten().tolist())
