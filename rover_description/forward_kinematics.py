import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from rclpy.time import Time
import math

# Constants from your URDF
# Wheel radius from <cylinder radius="0.11" ... />
WHEEL_RADIUS = 0.11  # meters

# Wheel base (distance between left and right wheels)
# From URDF origins: steer1(Y=0.291) and steer2(Y=-0.295)
# Distance = 0.29162 - (-0.29588) = 0.5875
WHEEL_BASE = 0.5875  # meters

# Joint names from your URDF
# Left side
WHEEL_LEFT_FRONT = 'wheel_4'
WHEEL_LEFT_REAR = 'wheel_1'
STEER_LEFT_FRONT = 'steer4'
STEER_LEFT_REAR = 'steer1'
# Right side
WHEEL_RIGHT_FRONT = 'wheel_3'
WHEEL_RIGHT_REAR = 'wheel_2'
STEER_RIGHT_FRONT = 'steer_3'
STEER_RIGHT_REAR = 'steer2'


class ForwardKinematicsNode(Node):
    """
    This node handles the forward kinematics for a 4-wheel rover.
    It subscribes to /cmd_vel and calculates the required wheel velocities
    to move the robot with the desired linear and angular speed,
    assuming a differential drive (skid-steer) model.
    
    It publishes the complete /joint_states for all 8 joints 
    (4 steering, 4 wheels).
    """
    def __init__(self):
        super().__init__('forward_kinematics_node')
        
        # Subscriber to /cmd_vel
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        
        # Publisher for /joint_states
        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)
        
        # Timer to publish joint states at a fixed rate (e.g., 50 Hz)
        self.timer_period = 0.02  # seconds (50 Hz)
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        # State variables
        self.target_linear_x = 0.0   # m/s
        self.target_angular_z = 0.0  # rad/s
        
        # We need to integrate position for 'continuous' joints
        self.pos_left = 0.0
        self.pos_right = 0.0
        
        self.last_time = self.get_clock().now()

        self.get_logger().info('Forward Kinematics Node has started.')

    def cmd_vel_callback(self, msg):
        """Stores the latest velocity command."""
        self.target_linear_x = msg.linear.x
        self.target_angular_z = msg.angular.z

    def timer_callback(self):
        """
        Calculates and publishes the JointState message.
        """
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9  # Delta time in seconds
        
        # --- Kinematic Calculation ---
        
        # 1. Calculate linear velocities (m/s) for left and right sides
        v_left_mps = self.target_linear_x - (self.target_angular_z * WHEEL_BASE / 2.0)
        v_right_mps = self.target_linear_x + (self.target_angular_z * WHEEL_BASE / 2.0)
        
        # 2. Convert linear velocities (m/s) to angular velocities (rad/s)
        # v = ω * r  =>  ω = v / r
        vel_left_radps = v_left_mps / WHEEL_RADIUS
        vel_right_radps = v_right_mps / WHEEL_RADIUS
        
        # 3. Integrate position (required for continuous joints)
        self.pos_left += vel_left_radps * dt
        self.pos_right += vel_right_radps * dt
        
        # --- Create and Publish JointState Message ---
        js_msg = JointState()
        js_msg.header.stamp = current_time.to_msg()
        
        # Set all 8 joint names in a specific order
        js_msg.name = [
            STEER_LEFT_FRONT, STEER_LEFT_REAR,
            STEER_RIGHT_FRONT, STEER_RIGHT_REAR,
            WHEEL_LEFT_FRONT, WHEEL_LEFT_REAR,
            WHEEL_RIGHT_FRONT, WHEEL_RIGHT_REAR
        ]
        
        # Set corresponding positions
        # Steering joints are fixed at 0 for differential drive
        js_msg.position = [
            0.0, 0.0,  # Left steering
            0.0, 0.0,  # Right steering
            self.pos_left, self.pos_left,    # Left wheels
            self.pos_right, self.pos_right  # Right wheels
        ]
        
        # Set corresponding velocities
        js_msg.velocity = [
            0.0, 0.0,  # Left steering
            0.0, 0.0,  # Right steering
            vel_left_radps, vel_left_radps,    # Left wheels
            vel_right_radps, vel_right_radps  # Right wheels
        ]
        
        self.publisher_.publish(js_msg)
        self.last_time = current_time


def main(args=None):
    rclpy.init(args=args)
    node = ForwardKinematicsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()