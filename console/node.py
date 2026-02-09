from rclpy.node import Node
from geometry_msgs.msg import Point 
from ap1_msgs.msg import VehicleSpeedStamped, TurnAngleStamped, MotorPowerStamped, TargetPathStamped, SpeedProfileStamped, FloatStamped, FloatStamped

import xml.etree.ElementTree as ET
from std_msgs.msg import String

class AP1SystemInterfaceNode(Node):
    def __init__(self):
        super().__init__('ap1_debug_ui')

        # publishers
        self.speed_pub = self.create_publisher(FloatStamped, '/ap1/control/target_speed', 10)
        self.target_location_pub = self.create_publisher(Point, '/ap1/control/target_location', 10)

        # subscribers
        self.speed_sub = self.create_subscription(FloatStamped, '/ap1/actuation/speed_actual', self.speed_callback, 10)
        self.turn_angle_sub = self.create_subscription(FloatStamped, '/ap1/actuation/turn_angle_actual', self.turn_angle_callback, 10)
        self.current_motor_power_sub = self.create_subscription(FloatStamped, '/ap1/control/motor_power', self.motor_power_callback, 10)
        self.path_sub = self.create_subscription(TargetPathStamped, '/ap1/planning/target_path', self.target_path_callback, 10)
        self.speed_profile = self.create_subscription(SpeedProfileStamped, '/ap1/planning/speed_profile', self.speed_profile_callback, 10)
        self.xml_sub = self.create_subscription(String, '/ap1/map/full_had_map', self.xml_callback, 10) # probably not needed anymore

        self.current_speed = 0.0 # m
        self.target_speed = 0.0 # m/s
        self.motor_power = 0.0 # [-1, 1]
        self.target_location = (0.0, 0.0) # m
        self.current_turn_angle = 0.0 # rads
        self.target_path = [] # waypoints
        self.speed_profile = [] # m/s
        self.features = [] # features from map

    def speed_callback(self, msg: FloatStamped):
        self.current_speed = msg.value

    def speed_profile_callback(self, msg: SpeedProfileStamped):
        self.speed_profile = msg.speeds

    def turn_angle_callback(self, msg: FloatStamped):
        self.current_turn_angle = msg.value
    
    def motor_power_callback(self, msg: FloatStamped):
        self.motor_power = msg.value

    def target_path_callback(self, msg: TargetPathStamped):
        self.target_path = msg.path
    
    def set_target_speed(self, speed: float):
        self.target_speed = speed

        # assemble msg
        msg = FloatStamped()
        msg.value = speed
        msg.header.stamp = self.get_clock().now().to_msg()

        # send out msg
        self.speed_pub.publish(msg)
        self.get_logger().info(f'Sent target speed: {speed}')

    def set_target_location(self, x: float, y: float):
        self.target_location = (x, y)
        msg = Point()
        msg.x = x
        msg.y = y 
        msg.z = 0.0
        self.target_location_pub.publish(msg)
        self.get_logger().info(f'Sent target location: ({x}, {y})')

    def xml_callback(self, msg: String):
        self.features.clear() # clear existing features
        root = ET.fromstring(msg.data)
        for feature in root.findall('Feature'): # parse features
            feature_type = feature.find('Type').text
            pos = feature.find('Position')
            x = float(pos.find('X').text)
            y = float(pos.find('Y').text)
            if feature_type in ['stop_sign', 'stop_line', 'traffic_light', 'yield_sign']: 
                self.features.append((feature_type, x, y)) 
            else: 
                self.features.append(('unknown', x, y))
        self.get_logger().info(f"Parsed {len(self.features)} features from XML")

# TEMPORARY
if __name__ == '__main__':
    import rclpy

    rclpy.init()
    node = AP1SystemInterfaceNode()

    try:
        node.set_target_speed(2)
    finally:
        # teardown
        node.destroy_node()
        rclpy.shutdown()
    