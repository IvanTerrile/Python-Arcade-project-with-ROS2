import time
import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, Float64
import math
from geometry_msgs.msg import Point
from simple_pid import PID

from sofar_printer_simulator_interface.srv import EndEffectorPosition

class MotorX(Node):

    def __init__(self):
        super ().__init__("motor_x_node")

        # Variables
        self.actual_pose_x = 0.0
        self.goal_x = 0.0
        #self.threshold = 0.1

        # Control gains
        # self.Kl = 0.5

        self.pid = PID(1, 0.1, 0.05)    #PID DEFAULT
        # self.pid = PID(5.0, 0.01, 0.05)    #PID MACCIO'

        # Update frequency (equivalent to publisher rate)
        self.dt = 0.01
        self.pid.sample_time = self.dt

        # Position error threshold and control saturation value
        self.control_clip_value = 1000
        self.threshold = 1

        self.motor_x_pub = self.create_publisher(Float64, "/motor_x", 10)
        self.ack_x_pub = self.create_publisher(Bool, "/ack_x", 10)
        self.controller_setpoint_sub = self.create_subscription(Point, "/controller_setpoint", self.callback_controller_setpoint, 10)

        self.motor_x_cli = self.create_client(EndEffectorPosition, '/end_effector_position')
        while not self.motor_x_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.request = EndEffectorPosition.Request()

        self.response = self.motor_x_ee()
        self.actual_pose_x = self.response.end_effector_position.x

    def callback_controller_setpoint(self, msg):
        self.goal_x = msg.x
        self.pid.reset()
        self.pid.setpoint = self.goal_x
        self.control_loop_timer = self.create_timer(self.dt, self.on_control_loop)

    def motor_x_ee(self):
        self.future = self.motor_x_cli.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    
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

def main(args=None):
    rclpy.init(args=args)
    motor_x = MotorX()

    rclpy.spin(motor_x)

    motor_x.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()