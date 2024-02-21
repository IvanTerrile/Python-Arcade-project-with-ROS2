import rclpy
from rclpy.node import Node
import time
from std_msgs.msg import Bool, Float64
import math
from geometry_msgs.msg import Point
from simple_pid import PID

class MotorX(Node):

    def __init__(self):
        super ().__init__("motor_x_node")

        # Log info
        self.get_logger().info("Motor X node is started")

        # Variables
        self.actual_pose_x = 0
        self.goal_x = 0
        self.threshold = 0.1

        # Control gains
        # self.Kl = 0.5

        self.pid = PID(1, 0.1, 0.05)

        # Update frequency (equivalent to publisher rate)
        self.dt = 0.01
        self.pid.sample_time = self.dt

        # Position error threshold and control saturation value
        self.control_clip_value = 100
        self.threshold = 1

        self.motor_x_pub = self.create_publisher(Float64, "/motor_x", 10)
        self.ack_x_pub = self.create_publisher(Bool, "/ack_x", 10)
        self.controller_setpoint_sub = self.create_subscription(Point, "/controller_setpoint", self.callback_controller_setpoint, 10)

    def callback_controller_setpoint(self, msg):
        self.goal_x = msg.x
        self.pid.reset()
        self.pid.setpoint = self.goal_x
        self.control_loop_timer = self.create_timer(self.dt, self.on_control_loop)

    def on_control_loop(self):
        control = self.pid(self.actual_pose_x)

         # Saturate control input if necessary (optional)
        if control > self.control_clip_value:
            control = self.control_clip_value
        elif control < -self.control_clip_value:
            control = -self.control_clip_value

        # Update position based on control
        self.actual_pose_x += control * self.dt

        # Publish it
        self.motor_x_pub.publish(Float64(data=self.actual_pose_x))

        if abs(self.actual_pose_x - self.pid.setpoint) < self.threshold:
            self.control_loop_timer.cancel()
            self.get_logger().info("Goal Reached!")
            self.ack_x_pub.publish(Bool(data=True))

    # def on_control_loop(self):
    #     # compute distance error
    #     e_d = math.sqrt((self.actual_pose_x - self.goal_x)**2)

    #     # exit loop if goal reached
    #     if e_d <= self.threshold:
    #         # cancel timer
    #         self.control_loop_timer.cancel()
    #         self.get_logger().info("Goal Reached!")
    #         # publish ack
    #         self.ack_x_pub.publish(Bool(data=True))

    #     # Compute control inputs
    #     lin_control = self.Kl * e_d

    #     if self.actual_pose_x > self.goal_x:
    #         self.actual_pose_x = self.actual_pose_x - lin_control
    #     else:
    #         self.actual_pose_x = self.actual_pose_x + lin_control

    #     # Publish it
    #     self.motor_x_pub.publish(Float64(data=self.actual_pose_x))
    

def main(args=None):
    rclpy.init(args=args)
    motor_x = MotorX()

    rclpy.spin(motor_x)

    motor_x.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()