import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D
from turtlesim.msg import Pose
import math

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
        self.goalBool = False
        self.v_max = 1      #constante para velocidade linear
        self.kw = 6     #constante para velocidade angular


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

        #armazenamento das variáveis de posição atual
        self.x = msg.x
        self.y = msg.y
        self.theta = msg.theta

        self.get_logger().info(f'pose info: x={self.x} y={self.y}, theta={self.theta}')
    

    def goal_callback(self, msg):

        #armazenamento das variáveis de posição "objetivo"
        self.x_goal = msg.x
        self.y_goal = msg.y
        self.theta_goal = msg.theta
        self.goalBool = True

        self.get_logger().info(f'goal info: x={self.x_goal} y={self.y_goal}, theta={self.theta_goal}')
    

    def pub_callback(self):
        
        if self.goalBool == False:
            self.get_logger().info('não recebi pose objetivo :/')
            return

        #cálculo do erro angular (alpha) e de posição (rô)
        positionError = math.sqrt((self.x_goal-self.x)**2 + (self.y_goal-self.y)**2)
        angularError = math.atan2(self.y_goal-self.y, self.x_goal-self.x) - self.theta

        #normalização do erro angular para que ele fique no intervalo [-pi, pi]
        angularError = (angularError + math.pi) % (2 * math.pi) - math.pi

        #cálculo das velocidades a serem publicadas
        linearVelo = self.v_max * math.tanh(positionError)
        angularVelo = self.kw * angularError

        if positionError < 0.01:    # para impedir que a tartaruga fique
            linearVelo = 0.0        #andando infinitamente
            angularVelo = 0.0
            self.get_logger().info('cheguei!!!')

        #criação e publicação da mensagem no tópico
        msg = Twist()
        msg.linear.x = linearVelo
        msg.angular.z = angularVelo
        self.publisher.publish(msg)

        self.get_logger().info(f'publishing: linear.x={linearVelo} angular.z={angularVelo}')


def main():
    rclpy.init(args=None)

    #criação do nó de controle da tartaruga
    turtleControl = TurtleControlNode()

    #loop de funcionamento do nó
    rclpy.spin(turtleControl)

    #fim do nó e do programa
    turtleControl.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()
