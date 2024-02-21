from threading import Thread
import time
import rclpy
from rclpy.node import Node

from terrile_nav_interface.srv import PathService, GripperService
from std_msgs.msg import Bool
from geometry_msgs.msg import Point

class RobotLogic(Node):

    def __init__(self):
        super().__init__("robot_logic_node")

        self.ack = True
        self.flag = 0

        self.path_cli = self.create_client(PathService, '/navigation/path')
        while not self.path_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('path service not available, waiting again...')
        self.path_req = PathService.Request()


        self.grasp_cli = self.create_client(GripperService, '/robot/grasp')
        while not self.grasp_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('grasp service not available, waiting again...')
        self.grasp_req = GripperService.Request()

        self.ack_sub = self.create_subscription(Bool, "/controller/ack", self.ack_callback, 10)
        self.next_waypoint_pub = self.create_publisher(Point, "/controller/goal", 10)

    def ack_callback(self, msg: Bool):
        self.ack = msg.data

    def path_request(self, idx: int):
        self.get_logger().info("Requesting path")
        self.path_req.path_idx.data = idx
        self.path_resp = self.path_cli.call(self.path_req)
        return self.path_resp.path
    
    def grasp_request(self):
        self.get_logger().info("Requesting gripper response")
        self.grasp_resp = self.grasp_cli.call(self.grasp_req)
        return self.grasp_resp.grasped.data

    def loop(self):
        self.get_logger().info("Starting loop")
        crate_path = self.path_request(0)
        # Iterate over waypoints
        for waypoint in crate_path[1:]:
            self.get_logger().info("Publishing waypoint")
            self.next_waypoint_pub.publish(waypoint)
            self.ack = False
            self.get_logger().info("Waiting for ack")
            while not self.ack:
                time.sleep(1)
            self.get_logger().info("Got ack")
        # Crate reached, try grasp...
        if self.grasp_request() == True:
            self.get_logger().info("Grasped")
            goal_path = self.path_request(1)
            for waypoint in goal_path[1:]:
                self.next_waypoint_pub.publish(waypoint)
                self.ack = False
                while not self.ack:
                    time.sleep(1)
            
def main (args=None):
    rclpy.init(args=args)
    logic = RobotLogic()
    
    # Spinning thread to make sure callbacks are executed
    spin_thread = Thread(target=rclpy.spin, args=(logic,))
    spin_thread.start()

    logic.loop()

    #rclpy.spin(logic)
    
    logic.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()