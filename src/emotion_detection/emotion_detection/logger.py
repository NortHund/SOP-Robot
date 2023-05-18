# Python libraries
import numpy as np
import sys
import time
import os

# ROS 2 packages
import rclpy 
from rclpy.node import Node 
from rclpy.executors import SingleThreadedExecutor
from rclpy.exceptions import ParameterNotDeclaredException
from ament_index_python.packages import get_package_share_directory

# ROS 2 services/messages
from std_msgs.msg import String



def main(args=None):
    # Initialize context
    rclpy.init(args=args) 

    try:
        executor = SingleThreadedExecutor()

        logger = Logger()
        if executor.add_node(logger) == False:
            logger.get_logger().fatal("[*] Failed to add a node to the executor")
            sys.exit(1)

        try:
            while rclpy.ok():
                # fetch and execute new callbacks
                executor.spin_once()

        except KeyboardInterrupt:
            logger.get_logger().info("exiting...")

        finally:
            # unnecessary kinda
            executor.shutdown(timeout_sec=1)	
            logger.destroy_node()

    except Exception as e:
        print(str(e))

    finally:
        rclpy.shutdown() # will also shutdown global executor



if __name__ == '__main__':
    main()
