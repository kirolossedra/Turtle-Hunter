import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import math
import time

class TurtleControlService(Node):

    def __init__(self):
        super().__init__('turtle_control_service')
        self.srv = self.create_service(Spawn, 'control_turtle', self.control_turtle_callback)
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.subscription = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.current_pose = None

    def pose_callback(self, msg):
        self.current_pose = msg
        self.get_logger().info("Receieved position x part is "+ str(msg.x))

    def control_turtle_callback(self, request, response):
        self.get_logger().info(f'Received request: x={request.x}, y={request.y}')

        target_x = request.x
        target_y = request.y

        while self.current_pose is None:
            rclpy.spin_once(self)

        current_x = self.current_pose.x
        current_y = self.current_pose.y
        current_theta = self.current_pose.theta

        self.get_logger().info(f'Current position: x={current_x}, y={current_y}, theta={current_theta}')

        delta_x = target_x - current_x
        delta_y = target_y - current_y
        desired_theta = math.atan2(delta_y, delta_x)

        

      
        
        twist = Twist()
        twist.linear.x = 0.0  # Move with a small speed
        self.get_logger().info(f'Calculated angle difference: {desired_theta}')
        if(desired_theta > current_theta):
            twist.angular.z = 2.0
        elif(desired_theta < current_theta):
            twist.angular.z = -2.0
        else :
            twist.angular.z = 0.0
        
        self.publisher.publish(twist)
        self.get_logger().info("Published Successfully")

        

        time.sleep(abs((desired_theta-current_theta)/2.0))


        twist = Twist()
        twist.linear.x = 4.0 # Move with a small speed
        twist.angular.z = 0.0
        self.publisher.publish(twist)
        self.get_logger().info("Published Successfully")
        distance = math.sqrt((target_x - current_x) ** 2 + (target_y - current_y) ** 2)


        start_time = time.time()

        while time.time() - start_time < (distance / 4.0):
            twist = Twist()
            twist.linear.x = 4.0 # Move with a small speed
            twist.angular.z = 0.0
            self.publisher.publish(twist)
            self.get_logger().info("Published Successfully")

        


        twist = Twist()
        twist.linear.x = 0.0  # Move with a small speed
        twist.angular.z = 0.0
        self.publisher.publish(twist)
        self.get_logger().info("Published Successfully")

        

        response.name = 'Turtle control successful'
        return response

def main(args=None):
    rclpy.init(args=args)
    turtle_control_service = TurtleControlService()
    rclpy.spin(turtle_control_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
