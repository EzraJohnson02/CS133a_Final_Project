import rclpy
from rclpy.node                 import Node
import numpy as np

from std_msgs.msg               import Float64
from finalproject.balldemo      import *
from finalproject.TrajectoryDual    import Trajectory1, Trajectory2
from finalproject.ProjectNodeDual   import ProjectNode
from finalproject.PointPublisher import *

#
#  Main Code
#
def main(args=None):
    # Initialize ROS.
    rclpy.init(args=args)
    node = GUINode('point', [0., 0., 0., -0.2, 0., 0.], 10)
    # Run until interrupted.
    node.run()
    p, v = node.getvalue()
    # Shutdown the node and ROS.
    node.destroy_node()

    # Initialize the generator node for 100Hz udpates, using the above
    # Trajectory class.
    generator = ProjectNode('generator', p, v, 100, Trajectory1, Trajectory2)

    # Spin, meaning keep running (taking care of the timer callbacks
    # and message passing), until interrupted or the trajectory ends.
    rclpy.spin(generator)

    # Shutdown the node and ROS.
    generator.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
