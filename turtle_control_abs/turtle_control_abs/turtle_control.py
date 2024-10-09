import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D
from turtlesim.msg import Pose

class TurtleControlNode(Node):

    def __init__(self):
        super().__init__('turtleControl')

        self.init_variables()
        self.init_subscribers()
        self.init_publisher()

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.pub_callback)
    

    def init_variables(self):
        self.x = 0
        self.y = 0
        self.theta = 0
        self.x_goal = 0
        self.y_goal = 0
        self.theta_goal = 0


    def init_publisher(self):
        self.publisher = self.create_publisher(Twist, "/abs/turtle1/cmd_vel", 10)

    
    def init_subscribers(self):

        self.create_subscription(
            Pose, 
            "/abs/turtle1/pose", 
            self.pose_callback, 
            10
        )
        
        self.create_subscription(
            Pose2D,
            "/abs/goal",
            self.goal_callback,
            10
        )
    
    
    def pose_callback(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.theta = msg.theta

        self.get_logger().info(f'pose info: x={self.x} y={self.y}, theta={self.theta}')
    

    def goal_callback(self, msg):
        self.x_goal = msg.x
        self.y_goal = msg.y
        self.theta_goal = msg.theta

        self.get_logger().info(f'goal info: x={self.x_goal} y={self.y_goal}, theta={self.theta_goal}')
    

    def pub_callback(self):
        pass

def main():
    rclpy.init(args=args)

    turtleControl = TurtleControlNode()

    rclpy.spin(turtleControl)

    turtleControl.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()
