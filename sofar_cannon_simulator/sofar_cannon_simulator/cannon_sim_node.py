from .lib.cannon_sim import CannonSim
import rclpy
from enum import Enum
from rclpy.node import Node

from std_msgs.msg import Float64, Bool, Empty
from geometry_msgs.msg import Point

from sofar_cannon_simulator_interface.srv import Ballistics, Targets

from ament_index_python.packages import get_package_share_directory


class CannonSimNode(Node):

    def __init__(self):
        super().__init__("cannon_sim_node")

        # Cannon simulator
        self.sim = CannonSim(get_package_share_directory("sofar_cannon_simulator"))
        self.sim.setup()

        # Publisher for cannon's shooting angle
        self.angle_pub = self.create_publisher(Float64, "/cannon/angle", 10)
        self.angle_pub_timer = self.create_timer(0.0333, self.on_state_timer_elapsed)
        
        # Subscriber for cannon's control
        self.create_subscription(Float64, "/cannon/control/rotation", self.on_rotation_speed_cmd, 10)

        # Subscriber to increase/decrease cannon's firing speed
        self.create_subscription(Bool, "/cannon/fire_speed", self.on_firing_speed_cmd, 10)
        
        # Subscriber for shooting cannonball
        self.create_subscription(Empty, "/cannon/shoot", self.on_shoot_cmd, 10)

        # Service for retrieving target locations
        self.create_service(Targets, "/world/targets", self.on_targets_srv_request)
        
        # Service for checking whether target is reachable (returns either a valid angle or -1 if unreachable)
        self.create_service(Ballistics, "/cannon/ballistics", self.on_ballistics_srv_request)
        

    def on_state_timer_elapsed(self):
        angle_msg = Float64()
        angle_msg.data = float(self.sim.get_cannon_angle())
        self.angle_pub.publish(angle_msg)

    def on_rotation_speed_cmd(self, msg: Float64):
        self.sim.rotate_cannon(msg.data)

    def on_firing_speed_cmd(self, msg: Bool):
        self.sim.set_cannon_fire_speed(msg.data)
        
    def on_shoot_cmd(self, msg: Empty):
        self.sim.shoot_cannon()

    def on_targets_srv_request(self, request: Targets.Request, response: Targets.Response):
        for target in self.sim.targets:
            target_msg = Float64()
            target_msg.data = float(target.center_x)
            response.targets.append(target_msg)
        return response
    
    def on_ballistics_srv_request(self, request: Ballistics.Request, response: Ballistics.Response):
        response.desired_angle.data = float(self.sim.find_target_angle(request.target.data))
        return response
    
  
def main(args=None):
    
    rclpy.init(args=args)
    sim_node = CannonSimNode()

    print("Press Ctrl+C to exit...")
    rclpy.spin(sim_node)

    sim_node.destroy_node()
    rclpy.shutdown()
    

# Script entry point
if __name__ == "__main__":
    main()