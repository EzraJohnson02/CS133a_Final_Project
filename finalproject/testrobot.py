'''hw6p5.py

   This is the skeleton code for HW6 Problem 5.  Please EDIT.

   This uses the inverse kinematics from Problem 4, but adds a more
   complex trajectory.

   Node:        /generator
   Publish:     /joint_states           sensor_msgs/JointState

'''

import rclpy
import numpy as np

from math import pi, sin, cos, acos, atan2, sqrt, fmod, exp

# Grab the utilities
from hw5code.GeneratorNode      import GeneratorNode
from hw5code.TransformHelpers   import *
from hw5code.TrajectoryUtils    import *
from scipy.optimize import fmin

# Grab the general fkin from HW5 P5.
from hw5code.KinematicChain     import KinematicChain

from std_msgs.msg import Float64


#
#   Trajectory Class
#
class Trajectory():
    # Initialization.
    def __init__(self, node):
        # Set up the kinematic chain object.
        self.chain = KinematicChain(node, 'world', 'ee_link', self.jointnames())

        # Define the various points.
        self.q0 = np.radians(np.array([0, 46.5675, -93.1349, 0, 0, 46.5675]).reshape((-1,1)))
        self.p0 = np.array([0.0, 0.7, 0.6]).reshape((-1,1))
        self.R0 = Roty(-np.pi/2)
        h = 30
        g = 10
        
        trajectoryf = lambda t: np.sqrt(np.square(10*np.sqrt(6) - 5 * np.abs(t)) +\
        				np.square(-10*np.sqrt(6) + 5 * np.abs(t)) +\
        				np.square(h - 0.5 * g * np.square(t)))
        t = fmin(trajectoryf, 0.)[0]

        x, y, z = 10 * np.sqrt(6) - 8 * np.abs(t), -10 * np.sqrt(6) + 8 * np.abs(t), h - 0.5 * g*t**2
        dist = np.sqrt(x**2 + y**2 + z**2)
        task_radius = 1.4
        if dist > task_radius:
            print("Ball is out of bounds!")
            self.pF = self.p0
        else:
            self.pF = np.array([x, y, z]).reshape((-1,1))
        self.T = t
        self.q = self.q0
        self.pd = self.p0
        self.Rd = self.R0

        self.lam = 20

        self.pub = node.create_publisher(Float64, '/condition', 10)

    # Declare the joint names.
    def jointnames(self):
        # Return a list of joint names FOR THE EXPECTED URDF!
        return ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']


    # Evaluate at the given time.  This was last called (dt) ago.
    def evaluate(self, t, dt):
        if t > self.T:
            return None
        (s0, s0dot) = goto(t, self.T, 0.0, 1.0)
        pd = self.p0 + (self.pF - self.p0) * s0
        vd = (self.pF - self.p0) * s0dot
        Rd = Roty(-np.pi/2)
        wd = -pi/2 * ey()

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

        L = 0.4

        Jbar = np.diag([1/L, 1/L, 1/L, 1, 1, 1]) @ J

        q = qlast + dt * qdot
        self.q = q
        condition = np.linalg.cond(Jbar)
        msg = Float64()
        msg.data = condition
        self.pub.publish(msg)
        self.q = q
        self.pd = pd
        self.Rd = Rd

        # Return the position and velocity as python lists.
        return (q.flatten().tolist(), qdot.flatten().tolist())


#
#  Main Code
#
def main(args=None):
    # Initialize ROS.
    rclpy.init(args=args)

    # Initialize the generator node for 100Hz udpates, using the above
    # Trajectory class.
    generator = GeneratorNode('generator', 100, Trajectory)

    # Spin, meaning keep running (taking care of the timer callbacks
    # and message passing), until interrupted or the trajectory ends.
    generator.spin()

    # Shutdown the node and ROS.
    generator.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
