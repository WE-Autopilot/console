from rclpy.node import Node
from std_msgs.msg import Float32 
from geometry_msgs.msg import Point 

class AP1SystemInterfaceNode(Node):
    def __init__(self):
        super().__init__('ap1_debug_ui')

        # publishers
        self.speed_pub = self.create_publisher(Float32, '/ap1/control/target_speed', 10)
        self.location_pub = self.create_publisher(Point, '/ap1/control/target_location', 10)

        # subscriber
        self.speed_sub = self.create_subscription(Float32, '/ap1/actuation/speed_actual', self.speed_callback, 10)
        self.turn_angle_sub = self.create_subscription(Float32, '/ap1/actuation/turn_angle_actual', self.turn_angle_callback, 10)

        self.current_speed = 0.0 # m
        self.target_speed = 0.0 # m/s
        self.target_location = (0.0, 0.0) # m
        self.current_turn_angle = 0.0 # rads

    def speed_callback(self, msg):
        self.current_speed = msg.data

    def turn_angle_callback(self, msg):
        self.current_turn_angle = msg.data
    
    def set_target_speed(self, speed: float):
        self.target_speed = speed
        msg = Float32()
        msg.data = speed 
        self.speed_pub.publish(msg)
        self.get_logger().info(f'Sent target speed: {speed}')

    def set_target_location(self, x: float, y: float):
        self.target_location = (x, y)
        msg = Point()
        msg.x = x
        msg.y = y 
        msg.z = 0.0
        self.location_pub.publish(msg)
        self.get_logger().info(f'Sent target location: ({x}, {y})')