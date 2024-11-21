import rclpy
from rclpy.node import Node
from math import atan
from std_msgs.msg import Int32MultiArray, Bool
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped

class AutonmousControl(Node):
    def __init__(self):
        super().__init__('autonomous_control')
        self.get_logger().info("Node Starting...")
        self.declare_parameters(
            namespace='',
            parameters=[
                ('use_system_deadman', rclpy.Parameter.Type.BOOL),
                ('ackermann_topic', rclpy.Parameter.Type.STRING),
                ('twist_topic', rclpy.Parameter.Type.STRING),
                ('traffic_light_topic', rclpy.Parameter.Type.STRING),
                ('system_deadman_topic', rclpy.Parameter.Type.STRING),
                ('wheelbase', rclpy.Parameter.Type.DOUBLE),
                ('frame_id', rclpy.Parameter.Type.STRING)
            ])
        # Setup Configuration
        self.use_system_deadman = self.get_parameter('use_system_deadman').value
        # Topic Variables
        self.ackermann_topic = self.get_parameter('ackermann_topic').value
        self.twist_topic = self.get_parameter('twist_topic').value
        self.traffic_light_topic = self.get_parameter('traffic_light_topic').value
        self.system_deadman_topic = self.get_parameter('system_deadman_topic').value

        # Robot Params
        self.wheelbase = self.get_parameter('wheelbase').value
        self.frame_id = self.get_parameter('frame_id').value

        # Message Creation
        self.ackermann_msg = AckermannDriveStamped()
        self.system_deadman_msg = Bool()
        
        # Deadman States
        self.system_deadman_state = 1 # System level deadman switch, 0 = not pressed, 1 = pressed. Car assumes traffic light is green (pressed) unless told otherwise
        self.light_status_dict = {0: 'red', 1: 'yellow', 2: 'green'}
        # Traffic Light Variables
        self.timestamp = 0
        self.traffic_light_id = 0
        self.traffic_light_status = 'green'

        # Subscriber for Twist, Traffic Light
        self.subscription = self.create_subscription(Twist, self.twist_topic, self.cmd_callback, 1)
        if self.use_system_deadman == True: # If using system deadman switch, subscribe to traffic light and execute traffic light callback
            self.traffic_light_subscription = self.create_subscription(Int32MultiArray, self.traffic_light_topic, self.traffic_light_callback, 1)
            self.system_deadman_publisher = self.create_publisher(Bool, self.system_deadman_topic, 1)
        
        # Publisher
        self.publisher = self.create_publisher(AckermannDriveStamped, self.ackermann_topic, 1)
        self.get_logger().info("Autonomous Driving Node Setup...")

    def convert_trans_rot_vel_to_steering_angle(self, v, omega):
        if omega == 0 or v == 0:
            return 0.0
        radius = v / omega
        steering_angle = float(atan(self.wheelbase / float(radius)))
        return steering_angle
    
    # Message Callbacks
    def traffic_light_callback(self, msg):
        # Get traffic light status if there is one published, store variables for use
        self.get_logger().info('Received Traffic Light Message...')
        self.traffic_light_msg = msg.data
        self.timestamp = self.traffic_light_msg[0]
        self.traffic_light_id = self.traffic_light_msg[1]
        # Update traffic light status and label numeric status with light color
        if self.traffic_light_msg[2] in self.light_status_dict:
            self.traffic_light_status = self.light_status_dict[self.traffic_light_msg[2]]
            self.get_logger().info(f'Timestamp: {self.timestamp}, Light ID: {self.traffic_light_id}, Light Status: {self.traffic_light_status}')
        else:
            self.get_logger().info(f'Unknown traffic light status: {self.traffic_light_msg[2]},\n Required: 1 = red, 2 = green, 3 = yellow')
        
        # Check System deadman state and update if necessary conditions are met 
        if self.traffic_light_status == 'red': # Red, System Deadman not Pressed
            self.system_deadman_state = 0
        elif self.traffic_light_status == 'green': # Green, System Deadman Pressed
            self.system_deadman_state = 1
        elif self.traffic_light_status == 'yellow': # Yellow, System Deadman Pressed
            self.system_deadman_state = 1
        else:
            self.get_logger().info(f'Unknown traffic light status: {self.traffic_light_status}')
        # publish message
        if self.system_deadman_state == 0:
            msg=False
        elif self.system_deadman_state == 1:
            msg=True
        self.system_deadman_msg.data = msg
        self.system_deadman_publisher.publish(self.system_deadman_msg)
        self.get_logger().info(f'Timestamp: {self.timestamp}, System Deadman: {self.system_deadman_state}')
    
    def create_stop_msg(self):
        self.ackermann_msg.header.stamp = self.get_clock().now().to_msg()
        self.ackermann_msg.header.frame_id = self.frame_id
        self.ackermann_msg.drive.steering_angle = 0.0
        self.ackermann_msg.drive.speed = 0.0

    def create_drive_msg(self, data):
        v = data.linear.x
        steering_angle = self.convert_trans_rot_vel_to_steering_angle(v, data.angular.z)
        steering_angle = float(steering_angle)
        
        # Set message values
        self.ackermann_msg.header.stamp = self.get_clock().now().to_msg()
        self.ackermann_msg.header.frame_id = self.frame_id
        self.ackermann_msg.drive.steering_angle = steering_angle
        self.ackermann_msg.drive.speed = v

    def cmd_callback(self, data):
        if self.system_deadman_state == 0: # System Deadman not Pressed (Light is Red) Publish stop ackermann msg
            #self.get_logger().info("System Deadman switch not pressed, not processing Autonomous Navigation message.")
            # Set message values to 0
            self.create_stop_msg()
            
        else: # System Deadman Pressed (Light is Green, Yellow, or no data coming through)
            # Create message values
            self.create_drive_msg(data)
        # publish message
        self.publisher.publish(self.ackermann_msg)
        
def main(args=None):  
    rclpy.init(args=args)
    autonomous_control = AutonmousControl()
    rclpy.spin(autonomous_control)
    autonomous_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__': 
    main()


