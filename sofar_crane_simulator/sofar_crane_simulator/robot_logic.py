import rclpy
from rclpy.node import Node

import time
from std_msgs.msg import Bool, Int64

class RobotLogic(Node):

    def __init__(self):
        super ().__init__("robot_logic_node")

        # Logic info
        self.get_logger().info("Robot logic node started!")

        self.ack_x = True
        self.ack_y = True

        self.stage_counter = 0

        # Publisher for next controller setpoint based on current pick/place stage
        self.next_stage_pub = self.create_publisher(Int64, "/next_stage", 10)
        self.create_timer(0.1, self.timer_callback)
        self.ack_x_sub = self.create_subscription(Bool, "/ack_x", self.callback_ack_x, 10)
        self.ack_y_sub = self.create_subscription(Bool, "/ack_y", self.callback_ack_y, 10)

    def callback_ack_x(self, msg):
        self.ack_x = msg.data

    def callback_ack_y(self, msg):
        self.ack_y = msg.data

    def timer_callback(self):
        if self.ack_x and self.ack_y:
            if self.stage_counter == 5:
                self.stage_counter = 0
            self.ack_x = False
            self.ack_y = False
            self.stage_counter += 1
            self.get_logger().info("Publishing next stage...")
            self.next_stage_pub.publish(Int64(data=self.stage_counter))
           

def main(args=None):
    rclpy.init(args=args)
    time.sleep(6)
    robot_logic = RobotLogic()

    rclpy.spin(robot_logic)

    robot_logic.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()