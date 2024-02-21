import rclpy
import time
from threading import Thread

from rclpy.node import Node
from std_msgs.msg import Bool, Float64, Empty

from sofar_cannon_simulator_interface.srv import Ballistics, Targets

class CannonLogic(Node):

    def __init__(self):
        super().__init__("robot_logic_node")

        self.idle = False

        self.targets_client = self.create_client(Targets, "/world/targets")
        while not self.targets_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('TargetsService not available, waiting...')

        self.ballistics_client = self.create_client(Ballistics, "/cannon/ballistics")
        while not self.ballistics_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('BallisticsService not available, waiting...')

        self.ack_sub = self.create_subscription(Bool, "/controller/ack", self.on_ack_received, 10)

        self.goal_pub = self.create_publisher(Float64, "/controller/goal", 10)

        self.cannon_fire_pub = self.create_publisher(Bool, "/cannon/fire_speed", 10)

        self.cannon_ballistics_pub = self.create_publisher(Empty, "/cannon/shoot", 10)


    def on_ack_received(self, msg: Bool):
        self.get_logger().info("Got ack from controller")
        self.idle = msg.data

    def get_list_of_target(self):
        req = Targets.Request()
        response = self.targets_client.call(req)
        return response.targets

    def ballistics(self, msg: Float64):
        req = Ballistics.Request()
        req.target = msg
        response = self.ballistics_client.call(req)
        return response.desired_angle

    def routine(self):
        # Get path from robot to crate
        list_of_target = self.get_list_of_target()
        self.get_logger().info("Got list of targets: {}".format(list_of_target))

        for i in range(len(list_of_target)):
            self.get_logger().info("Target: {}".format(list_of_target[i]))
            # angle = self.ballistics(list_of_target[i]).data
            self.get_logger().info("Desired angle: {}".format(self.ballistics(list_of_target[i]).data))

            while self.ballistics(list_of_target[i]).data < 0.0:
                self.get_logger().info("Target {} unreachable".format(list_of_target[i]))
                self.cannon_fire_pub.publish(Bool(data=True))
                self.cannon_ballistics_pub.publish(Empty())
        
            while self.ballistics(list_of_target[i]).data >= 0.0 and self.ballistics(list_of_target[i]).data <= 0.5:
                self.cannon_fire_pub.publish(Bool(data=True))
                self.cannon_ballistics_pub.publish(Empty())

            if self.ballistics(list_of_target[i]).data > 0.5:
                self.goal_pub.publish(self.ballistics(list_of_target[i]))
                if self.idle:
                    self.get_logger().info("Idle")
                    self.cannon_ballistics_pub.publish(Empty())
            

def main(args=None):
    rclpy.init(args=args)

    # Wait for other nodes to init properly
    time.sleep(3)

    # Create and spin controller node
    logic = CannonLogic()
    
    # Spinning thread to make sure callbacks are executed
    spin_thread = Thread(target=rclpy.spin, args=(logic,))
    spin_thread.start()

    # Start logic node routine
    logic.routine()

    # On shutdown..
    logic.get_logger().info("Shutdown logic node...")
    logic.destroy_node()
    rclpy.shutdown()

# Script entry point
if __name__ == '__main__':
    main()