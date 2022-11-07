import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist,  TransformStamped, PoseStamped, Pose
from turtle_brick_interfaces.msg import Tilt
import math


from std_msgs.msg import String

class Flip(Node):

    def __init__(self):
        super().__init__('flip')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('body_width', 1.0),
                ('total_width', 1.3),
                ('body_length', 2.0),
                ('body_height', 0.5),
                ('total_height', 1.2),
                ('caster_radius', 0.15),
            ]#change
        )
        #change
        self.bw = self.get_parameter("body_width").get_parameter_value().double_value
        self.tw = self.get_parameter("total_width").get_parameter_value().double_value
        self.bl = self.get_parameter("body_length").get_parameter_value().double_value
        self.bh = self.get_parameter("body_height").get_parameter_value().double_value
        self.th = self.get_parameter("total_height").get_parameter_value().double_value
        self.rcaster = self.get_parameter("caster_radius").get_parameter_value().double_value
        # initializing publishers
        qos_profile = QoSProfile(depth=10)
        self.joint_publisher_ = self.create_publisher(JointState, '/joint_states', qos_profile)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # Static broadcasters publish on /tf_static.
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.broadcaster = TransformBroadcaster(self)
        # world to odom transform
        world_odom_tf = TransformStamped()
        world_odom_tf.header.stamp = self.get_clock().now().to_msg()
        world_odom_tf.header.frame_id = "world"
        world_odom_tf.child_frame_id = "odom"
        world_odom_tf.transform.translation.x = 0.0
        world_odom_tf.transform.translation.y = 0.0
        self.static_broadcaster.sendTransform(world_odom_tf)
        # velocity
        self.vx = 11.0
        self.direction = 1
        self.lastdir = self.direction
        # create the odom to baselink transform
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.odom_base_link = TransformStamped()
        self.odom_base_link.header.frame_id = "odom"
        self.odom_base_link.child_frame_id = "base_link"
        self.odom_base_link.transform.translation.z = self.th
        # odommetry object
        self.odom = Odometry()
        self.odom.header.frame_id = "odom"
        self.odom.child_frame_id = "base_link"

        self.joint_state = JointState()
        self.joint_state.header.frame_id = 'base_link'
        # cmd_vel object
        # create pose object
        # Create a timer to do movements
        self.i = 0
        self.freq = 50.0
        self.period = 1 / self.freq
        self.tmr = self.create_timer(self.period, self.timer_callback)

    def timer_callback(self):
        # update odom
        self.odom.header.stamp = self.get_clock().now().to_msg()
        if self.i==50:
            self.direction=self.direction*-1
            self.lastdir = self.direction
        elif self.i==125:
            self.direction=0
        elif self.i==225:
            self.direction = self.lastdir
            self.i = 0
        self.odom.twist.twist.linear.x = self.direction*self.vx
        # broadcasting and publishing
        self.odom_pub.publish(self.odom)
        self.cmd_vel_pub .publish(self.odom.twist.twist)        
        self.i = self.i+1
        


def main(args=None):
    rclpy.init(args=args)
    flip = Flip()
    rclpy.spin(flip)
    rclpy.shutdown()


if __name__ == '__main__':
    main()