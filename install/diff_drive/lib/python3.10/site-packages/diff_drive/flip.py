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
        # positions
        self.currentx = 0.0
        self.currenty = 0.0
        self.currentomega = 0.0
        self.goalx = 0.0
        self.goaly = 0.0
        self.distx = 0.0
        self.disty = 0.0
        # velocity
        self.vx = 0.0
        self.vy = 0.0
        self.dx = 0.0
        self.dy = 0.0
        self.dtheta = 0.0
        self.stem_ang = 0.0
        # create the odom to baselink transform
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.odom_base_link = TransformStamped()
        self.odom_base_link.header.frame_id = "odom"
        self.odom_base_link.child_frame_id = "base_link"
        self.odom_base_link.transform.translation.z = 2 * self.rwheel + 0.15 + 0.25
        # joint state object
        self.joint_state = JointState()
        self.joint_state.header.frame_id = 'base_link'
        self.wheel_ang = 0.0
        self.direction_ang = 0.0
        self.tilt_ang = 0.0
        self.tilt_angf = 0.0
        # odommetry object
        self.odom = Odometry()
        self.odom.header.frame_id = "odom"
        self.odom.child_frame_id = "base_link"
        # cmd_vel object
        self.twist = Twist()
        # create pose object
        self.robo_pose = Pose()
        # Create a timer to do movements
        self.freq = 100.0
        self.period = 1 / self.freq
        self.tmr = self.create_timer(self.period, self.timer_callback)

    def timer_callback(self):
        time = self.get_clock().now().to_msg()
        # update odom
        self.odom.header.stamp = self.get_clock().now().to_msg()
        self.odom.pose.pose.position.x = self.currentx
        self.odom.pose.pose.position.y = self.currenty
        self.odom.twist.twist.linear.x = self.vx
        self.odom.twist.twist.linear.y = self.vy
        # updating joint states
        self.joint_state.header.stamp = self.get_clock().now().to_msg()
        self.joint_state.name = ['base_to_L_front_wheel', 'base_to_R_front_wheel', 'base_to_top_caster_wheel','base_to_bottom_caster_wheel']
        self.joint_state.position = [self.L_wheel_ang, self.R_wheel_ang, self.flip]
        # update twist
        self.twist.linear.x = self.vx
        self.twist.linear.y = self.vy
        # update robot pose
        self.robo_pose.position.x = self.currentx
        self.robo_pose.position.y = self.currenty
        self.robo_pose.position.z = self.height
        self.robo_pose.orientation.w = self.tilt_ang
        # update transforms
        self.odom_base_link.transform.translation.x = self.currentx
        self.odom_base_link.transform.translation.y = self.currenty
        self.odom_base_link.header.stamp = time
        # broadcasting and publishing
        self.joint_publisher_.publish(self.joint_state)
        self.odom_pub.publish(self.odom)
        self.vel_pub.publish(self.twist)
        self.robot_pub.publish(self.robo_pose)
        self.broadcaster.sendTransform(self.odom_base_link)
        # moving the robot to goal pose
        self.currentx += self.dx
        self.currenty += self.dy
        if abs(self.currentx-self.goalx) <= 0.03:
            self.vx = 0.0
            self.dx = 0.0
        if abs(self.currenty-self.goaly) <= 0.03:
            self.vy = 0.0
            self.dy = 0.0
        # move the wheel and stem to face direction and roll
        if self.vx != 0.0 and self.vy != 0.0:
            self.wheel_ang += self.dtheta
        if abs(self.stem_ang - self.direction_ang) >= 0.2:
            if self.stem_ang - self.direction_ang >= 0:
                self.direction_ang += 0.1
                self.get_logger().info('stem angle: "%s"' % (self.direction_ang))
            else:
                self.direction_ang -= 0.1
                self.get_logger().info('stem angle: "%s"' % (self.direction_ang))
        # tilt if tilt is called
        if self.tilt_ang != 0.0 or self.tilt_angf != 0.0:
            if abs(self.tilt_ang - self.tilt_angf) >= 0.01:
                if self.tilt_angf - self.tilt_ang < 0.0:
                    self.tilt_ang -= 0.005
                elif self.tilt_angf - self.tilt_ang >= 0.0:
                    self.tilt_ang += 0.005
            elif self.tilt_angf == 0.0:
                self.tilt_ang = 0.0
            else:
                self.tilt_angf = 0.0


def main(args=None):
    rclpy.init(args=args)
    flip = Flip()
    rclpy.spin(flip)
    rclpy.shutdown()


if __name__ == '__main__':
    main()