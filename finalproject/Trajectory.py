import numpy as np
from math import pi, sin, cos, acos, atan2, sqrt, fmod, exp
from finalproject.TransformHelpers     import *
from finalproject.TrajectoryPlanner    import *

# Grab the utilities
from hw5code.TransformHelpers   import *
from hw5code.TrajectoryUtils    import *
from scipy.optimize import fmin

# Grab the general fkin from HW5 P5.
from hw5code.KinematicChain     import KinematicChain
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
            print(f"Ball is out of bounds! Distance {distance}")
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
        self.q = self.q0
        self.pd = self.p0
        self.Rd = self.R0 @ Roty(-pi/2) @ Rote(self.e_paddle, -self.theta_paddle)

        self.lam = 20

        self.pub = node.create_publisher(Float64, '/condition', 10)

    def set_goal(self, p_ball, v_ball, a_ball):
        min_time = 0.2
        tp = TrajectoryPlanner(p_ball, v_ball, a_ball, min_time)

        t = tp.calculate_min_time()
        catch_position = tp.position(t)

        distance = np.linalg.norm(catch_position)
        task_radius = 1.4
        self.q0 = self.q
        self.p0 = self.pF
        self.R0 = Reye()

        if distance > task_radius:
            print(f"Ball is out of bounds! Distance {distance}")
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

        qlast = self.q
        pdlast = self.pd
        Rdlast = self.Rd

        (p, R, Jv, Jw) = self.chain.fkin(qlast)
        vr = vd + self.lam * ep(pdlast, p)
        wr = wd + self.lam * eR(Rdlast, R)
        J = np.vstack((Jv, Jw))
        xrdot = np.vstack((vr, wr))

        M, N = J.shape
        gamma = 0.1
        lam_s = 10
        Jp = np.transpose(J) @ np.linalg.pinv(J @ np.transpose(J) + gamma ** 2 * np.eye(M))
        qdiff = np.array([0, 0, 0, np.pi/2 - qlast[5, 0], 0, 0]).reshape(6, 1)
        qdot_second = lam_s * qdiff
        qdot = Jp @ xrdot + (np.eye(N) - Jp @ J) @ qdot_second

        q = qlast + dt * qdot
        self.q = q

        self.q = q
        self.pd = pd

        # Return the position and velocity as python lists.
        return (q.flatten().tolist(), qdot.flatten().tolist())
