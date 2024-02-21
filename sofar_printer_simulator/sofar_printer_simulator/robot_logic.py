import rclpy
from rclpy.node import Node
import time

from std_msgs.msg import Bool
from geometry_msgs.msg import Point
from sofar_printer_simulator_interface.srv import ShapeService

class RobotLogic(Node):

    def __init__(self):
        super().__init__("robot_logic_node")
        self.get_logger().info("Robot logic node up and running")

        # Declare parameters
        self.declare_parameter("radius", 0.0)
        self.declare_parameter("n_vertices", 0)

        # Acknowledgements
        self.ack_x = True
        self.ack_y = True

        self.shape_cli = self.create_client(ShapeService, 'shape_srv')
        while not self.shape_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.request = ShapeService.Request()

        self.response = self.shape_request()
        self.get_logger().info("Received x and y vertices")
        self.get_logger().info("x: " + str(self.response.x_vertices))
        self.get_logger().info("y: " + str(self.response.y_vertices))

        self.ack_x_sub = self.create_subscription(Bool, "/ack_x", self.ack_x_callback, 10)
        self.ack_y_sub = self.create_subscription(Bool, "/ack_y", self.ack_y_callback, 10)
        self.next_waypoint_pub = self.create_publisher(Point, "/next_waypoint", 10)
        self.draw_pub = self.create_publisher(Bool, "/draw", 10)

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.flag = -1

    def ack_x_callback(self, msg):
        self.ack_x = msg.data

    def ack_y_callback(self, msg):
        self.ack_y = msg.data

    def shape_request(self):
        self.get_logger().info("Requesting shape")
        self.request.radius = self.get_parameter("radius").get_parameter_value().double_value
        self.request.n_vertices = self.get_parameter("n_vertices").get_parameter_value().integer_value
        self.future = self.shape_cli.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    
    def timer_callback(self):
        if self.ack_x and self.ack_y:
            self.flag += 1
            self.get_logger().info("flag: " + str(self.flag))
            self.ack_x = False
            self.ack_y = False
            self.get_logger().info("Sending next waypoint")
            next_waypoint = Point()
            if self.flag > 0 and self.flag <= len(self.response.x_vertices):
                self.draw_pub.publish(Bool(data=True))
            else:
                self.draw_pub.publish(Bool(data=False))
            if self.flag == len(self.response.x_vertices):
                next_waypoint.x = self.response.x_vertices[0]
                next_waypoint.y = self.response.y_vertices[0]
            elif self.flag > len(self.response.x_vertices):
                self.timer.cancel()
            else:
                next_waypoint.x = self.response.x_vertices[self.flag]
                next_waypoint.y = self.response.y_vertices[self.flag]
            self.next_waypoint_pub.publish(next_waypoint)
            self.get_logger().info("Done sending waypoints")
        

def main(args=None):
    rclpy.init(args=args)
    time.sleep(3)
    robot_logic_node = RobotLogic()
    rclpy.spin(robot_logic_node)
    robot_logic_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()