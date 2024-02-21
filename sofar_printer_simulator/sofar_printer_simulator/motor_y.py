import time
import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, Float64
import math
from geometry_msgs.msg import Point

from sofar_printer_simulator_interface.srv import EndEffectorPosition  

from simple_pid import PID

class MotorY(Node):

    def __init__(self):
        super ().__init__("motor_x_node")

        # Variables
        self.actual_pose_y = 0.0
        self.goal_y = 0.0
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

        self.motor_y_pub = self.create_publisher(Float64, "/motor_y", 10)
        self.ack_y_pub = self.create_publisher(Bool, "/ack_y", 10)
        self.controller_setpoint_sub = self.create_subscription(Point, "/controller_setpoint", self.callback_controller_setpoint, 10)

        self.motor_y_cli = self.create_client(EndEffectorPosition, '/end_effector_position')
        while not self.motor_y_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.request = EndEffectorPosition.Request()

        self.response = self.motor_y_ee()
        self.actual_pose_y = self.response.end_effector_position.y

    def callback_controller_setpoint(self, msg):
        self.goal_y = msg.y
        self.pid.reset()
        self.pid.setpoint = self.goal_y
        self.control_loop_timer = self.create_timer(0.01, self.on_control_loop)

    def motor_y_ee(self):
        self.future = self.motor_y_cli.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def on_control_loop(self):
        control = self.pid(self.actual_pose_y)

         # Saturate control input if necessary (optional)
        if control > self.control_clip_value:
            control = self.control_clip_value
        elif control < -self.control_clip_value:
            control = -self.control_clip_value

        # Update position based on control
        self.actual_pose_y += control * self.dt

        # Publish it
        self.motor_y_pub.publish(Float64(data=self.actual_pose_y))

        if abs(self.actual_pose_y - self.pid.setpoint) < self.threshold:
            self.control_loop_timer.cancel()
            self.get_logger().info("Goal Reached!")
            self.ack_y_pub.publish(Bool(data=True))
    

def main(args=None):
    rclpy.init(args=args)
    motor_y = MotorY()

    rclpy.spin(motor_y)

    motor_y.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()