
import time
import rclpy
from rclpy.node import Node

from sofar_printer_simulator_interface.srv import ShapeService
import math

class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.get_logger().info('Shape Service node up and running')
        self.srv = self.create_service(ShapeService, 'shape_srv', self.compute_shape_callback)
        

    def compute_shape_callback(self, request, response):
        response.x_vertices = [0.0] * request.n_vertices  
        response.y_vertices = [0.0] * request.n_vertices  

        for i in range(request.n_vertices): 
            response.x_vertices[i]=(request.radius * math.cos((2 * math.pi * i )/ (request.n_vertices)))
            response.y_vertices[i]=(request.radius * math.sin((2 * math.pi * i )/ (request.n_vertices)))
           
        return response
            

def main():
    rclpy.init()
    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()

if __name__ == '__main__':
    main()