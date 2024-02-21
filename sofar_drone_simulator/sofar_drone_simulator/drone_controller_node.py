import rclpy
import math
import numpy as np

from rclpy.node import Node
from simple_pid import PID
from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import Pose2D, Point, Twist
from sofar_drone_simulator_interface.msg import DroneState

class RobotController(Node):

    def __init__(self):
        super().__init__("drone_controller_node")

        self.goal = Point()
        self.current_state = DroneState()
        self.dt = 0.05

        # Define PID controllers for linear velocity
        self.lin_pid = PID(5.0, 0.001, 0.2)
        
        # Controllers update frequency (equivalent to publisher rate)
        self.lin_pid.sample_time = self.dt

        # Position and angular errors threshold and control saturation value
        self.lin_threshold = 1.0
        self.lin_control_clip_value = 3.0

        self.goal_sub = self.create_subscription(Point, "/controller/goal", self.set_new_target_pose, 10)
        self.state_sub = self.create_subscription(DroneState, "/drone/state", self.pose_callback, 10)

        self.thrust_pub = self.create_publisher(Float64, "/drone/control/thrust", 10)
        self.speed_pub = self.create_publisher(Float64, "/drone/control/speed", 10)

        self.get_logger().info("Controller module initialized!")
        self.get_logger().info("Ready to received target setpoints...")

    # Pose callback to keep track of robot's current position
    def pose_callback(self, msg: DroneState):
        self.current_state = msg
        self.get_logger().info("Current state: (position: {0}, velocity: {1})".format(msg.position, msg.velocity))

    # Reset PID target points and start new control loop timer
    def set_new_target_pose(self, msg: Point):
        self.get_logger().info('Received new target position (x: {0}, y: {1})'.format(msg.x, msg.y))

        self.goal = msg.x
        self.lin_pid.reset()
        self.lin_pid.setpoint = self.goal

        # Start timer for control loop callback
        self.timer = self.create_timer(self.dt, self.control_loop_callback)

    # Control loop cycle callback
    def control_loop_callback(self):
        control = self.lin_pid(self.current_state.position.x)

         # Saturate control input if necessary (optional)
        if control > self.lin_control_clip_value:
            control = self.lin_control_clip_value
        elif control < -self.lin_control_clip_value:
            control = -self.lin_control_clip_value

         # Update position based on control
        self.current_state.position.x += control * self.dt

        # Publish it
        self.speed_pub.publish(Float64(data=self.current_state.position.x))

        if abs(self.current_state.position.x - self.lin_pid.setpoint) < self.lin_threshold:
            self.timer.cancel()
            self.get_logger().info("Goal Reached!")

def main(args=None):
    rclpy.init(args=args)

    # Create and spin controller node
    drone_controller_node = RobotController()
    rclpy.spin(drone_controller_node)

    # On shutdown..
    drone_controller_node.get_logger().info("Shutdown controller node...")
    drone_controller_node.destroy_node()
    rclpy.shutdown()

# Script entry point
if __name__ == '__main__':
    main()