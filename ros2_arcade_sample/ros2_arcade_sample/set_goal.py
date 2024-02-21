import rclpy
from rclpy.node import Node

from ros2_arcade_sample_interface2.msg import Goal

class SetGoal(Node):
    
    def __init__(self):
        super().__init__('set_goal')
        self.set_goal_pub_ = self.create_publisher(Goal, '/goal', 10)
        
        self.goal = Goal()

        # Declere Parameters
        self.declare_parameter("goal_x", 0)
        self.declare_parameter("goal_y", 0)

    def timer_callback(self):
        self.set_goal_pub_.publish(self.goal)

    def set(self):
        # Get Parameters
        my_goal_x = self.get_parameter("goal_x").get_parameter_value().integer_value
        my_goal_y = self.get_parameter("goal_y").get_parameter_value().integer_value
        self.goal.x.data = my_goal_x 
        self.goal.y.data = my_goal_y

        # Set Parameters
        # my_new_goal_x = rclpy.Parameter(
        #     'my_new_goal_x',
        #     rclpy.Parameter.Type.INTEGER,
        #     12
        # )
        # my_new_goal_y = rclpy.Parameter(
        #     'my_new_goal_y',
        #     rclpy.Parameter.Type.INTEGER,
        #     75
        # )
        # all_new_parameters = [my_new_goal_x, my_new_goal_y]
        # self.set_parameters(all_new_parameters)

        self.timer = self.create_timer(0.5, self.timer_callback)
   
def main(args=None):
    rclpy.init(args=args)

    set_goal = SetGoal()
    set_goal.set()
    rclpy.spin(set_goal)

    set_goal.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()