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



        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
    
    def init_variables(self):
        self.x = 0
        self.y = 0


    def timer_callback(self):
        pass
    
    def init_publisher(self):
        self.publisher = self.create_publisher(Twist, "/abs/turtle1/cmd_vel", 10)
    
    def init_subscribers(self):
        
        self.create_subscription(
                Pose, 
                "turtlesim/msg/Pose", 
                self.pose_callback, 
                10)
        
    
    def pose_callback(self):
        pass
    
    def goal_callback(self):
        pass
    
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
