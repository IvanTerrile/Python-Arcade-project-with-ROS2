import time
import rclpy
import math
import numpy as np

from rclpy.node import Node
from simple_pid import PID
from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import Pose2D, Point, Twist

class CannonController(Node):

    def __init__(self):
        super().__init__("cannon_controller_node")

        self.current_angle = 0.0
        self.dt = 0.01

        # Define PID controllers for linear velocity and rotation
        self.ang_pid = PID(50, 0.1, 0.2)
        # valori di default 1, 0.1, 0.05
        # 0.75, 0.01, 0.05
        
        # Controllers update frequency (equivalent to publisher rate)
        self.ang_pid.sample_time = self.dt

        # Position and angular errors threshold and control saturation value
        self.ang_threshold = 0.01
        
        # Cannon angle subscriber
        self.pose_sub = self.create_subscription(Float64, "/cannon/angle", self.angle_callback, 10)

        # Cannon next goal subscriber
        self.goal_sub = self.create_subscription(Float64, "/controller/goal", self.set_new_target, 10)

        # Cannon control rotation publisher
        self.rotation_pub = self.create_publisher(Float64, "/cannon/control/rotation", 10)
        
        # Ack publisher
        self.ack_pub = self.create_publisher(Bool, "/controller/ack", 10)

        self.get_logger().info("Controller module initialized!")
        self.get_logger().info("Ready to received target setpoints...")

    # Angel callback to keep track of robot's current angle
    def angle_callback(self, msg: Float64):
        self.current_angle = msg.data
        # self.get_logger().info("Current angle: {0}".format(self.current_angle))

    # Reset PID target points and start new control loop timer
    def set_new_target(self, msg: Float64):
        self.get_logger().info('Received new target: {0}'.format(msg))

        # Reset PID internals to clear previous errors
        self.ang_pid.reset()

        self.goal = msg.data

        self.ang_pid.setpoint = self.goal

        # Start timer for control loop callback
        self.timer = self.create_timer(self.dt, self.control_loop_callback)

    # Control loop cycle callback
    def control_loop_callback(self):

        if abs(self.current_angle - self.goal) < self.ang_threshold:
            self.get_logger().info('Angle reached')
            self.timer.cancel()
            # Publish ack message to navigator
            ack_msg = Bool()
            ack_msg.data = True
            self.ack_pub.publish(ack_msg)
            exit()
        else:
            # Compute rotation control based on target rotation
            ang_control = self.ang_pid(self.current_angle) * self.dt
            # Publish command velocity message
            cmd_vel = Float64()
            cmd_vel.data = ang_control
            self.rotation_pub.publish(cmd_vel)
 
def main(args=None):
    rclpy.init(args=args)
    time.sleep(5)

    # Create and spin controller node
    controller_node = CannonController()
    # max 1.57
    #controller_node.set_new_target(0.75)

    rclpy.spin(controller_node)

    # On shutdown..
    controller_node.get_logger().info("Shutdown controller node...")
    controller_node.destroy_node()
    rclpy.shutdown()

# Script entry point
if __name__ == '__main__':
    main()