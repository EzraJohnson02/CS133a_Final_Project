'''hw6p5.py

   This is the skeleton code for HW6 Problem 5.  Please EDIT.

   This uses the inverse kinematics from Problem 4, but adds a more
   complex trajectory.

   Node:        /generator
   Publish:     /joint_states           sensor_msgs/JointState

'''

import rclpy
from rclpy.node                 import Node
import numpy as np

from std_msgs.msg               import Float64
from finalproject.balldemo      import *
from finalproject.Trajectory    import Trajectory
from finalproject.ProjectNode   import ProjectNode
from finalproject.PointPublisher import *

#
#  Main Code
#
def main(args=None):
    # Initialize ROS.
    rclpy.init(args=args)
    node = GUINode('point', [0., 0., 0.], 10)
    # Run until interrupted.
    node.run()
    p = node.getvalue()
    # Shutdown the node and ROS.
    node.destroy_node()

    # Initialize the generator node for 100Hz udpates, using the above
    # Trajectory class.
    generator = ProjectNode('generator', p, 100, Trajectory)

    # Spin, meaning keep running (taking care of the timer callbacks
    # and message passing), until interrupted or the trajectory ends.
    rclpy.spin(generator)

    # Shutdown the node and ROS.
    generator.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
