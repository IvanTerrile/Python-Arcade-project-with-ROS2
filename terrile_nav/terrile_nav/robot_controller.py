import time
from rclpy.node import Node
import rclpy
import math

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Bool

class RobotController(Node):

    def __init__(self):
        super().__init__("robot_controller_node")

        # Variables
        self.robot_pose = None
        self.goal_x = 0
        self.goal_y = 0
        self.ang_threshold = 0.005
        self.lin_threshold = 0.1

        # Control gains
        self.Kl = 0.05
        self.Ka = 5.0

        # Pub and sub
        self.pose_sub = self.create_subscription(Pose2D, "/robot/pose", self.on_pose_received, 10)
        self.goal_sub = self.create_subscription(Point, "/controller/goal", self.on_goal_received, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, "/robot/cmd_vel", 10)
        self.ack_pub = self.create_publisher(Bool, "/controller/ack", 10)
    
    def on_pose_received(self, msg: Pose2D):
        self.robot_pose = msg

    def on_goal_received(self, msg: Point):
        self.set_goal(msg.x, msg.y)

    def set_goal(self, x: float, y: float):
        self.goal_x = x
        self.goal_y = y
        self.control_loop_timer = self.create_timer(0.01, self.on_control_loop)

    def on_control_loop(self):
        # compute distance error
        e_d = math.sqrt((self.robot_pose.x - self.goal_x)**2 + (self.robot_pose.y - self.goal_y)**2)
        # compute angular error
        e_a = math.atan2(self.goal_y - self.robot_pose.y, self.goal_x - self.robot_pose.x) - self.robot_pose.theta

        # normalize angular error
        if e_a > math.pi:
            e_a -= 2*math.pi
        elif e_a < -math.pi:
            e_a += 2*math.pi

        # Build twist msg
        cmd_vel = Twist()

        # exit loop if goal reached
        if e_d <= self.lin_threshold:
            # cancel timer
            self.control_loop_timer.cancel()
            self.get_logger().info("Goal Reached!")
            self.ack_pub.publish(Bool(data=True))
        elif abs(e_a) <= self.ang_threshold:
            lin_control = self.Kl * e_d
            cmd_vel.linear.x = lin_control
            # Publish it
            self.cmd_vel_pub.publish(cmd_vel)
        else:
            # Compute control inputs
            ang_control = self.Ka * e_a
            cmd_vel.angular.z = ang_control
            # Publish it
            self.cmd_vel_pub.publish(cmd_vel)

        

def main():
    rclpy.init(args = None)
    
    robot_controller = RobotController()

    rclpy.spin(robot_controller)

    robot_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()