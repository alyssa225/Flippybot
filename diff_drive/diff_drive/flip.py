import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class Flip(Node):
    """
    Flips car robot in Ignition Gazebo.

    PUBLISHERS:
    ----------
    cmd_vel_pub (type: Twist): publishes changes in velocities

    """

    def __init__(self):
        """Create initialize variables, publisher, start timer."""
        super().__init__('flip')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.vx = 11.0
        self.direction = -1
        self.lastdir = self.direction
        self.odom = Odometry()
        self.odom.header.frame_id = "/odom"
        self.odom.child_frame_id = "/base_link"
        self.i = 0
        self.freq = 50.0
        self.period = 1 / self.freq
        self.tmr = self.create_timer(self.period, self.timer_callback)

    def timer_callback(self):
        """Change in velocity for flipping robot."""
        self.odom.header.stamp = self.get_clock().now().to_msg()
        if self.i == 75:
            self.direction = self.direction * -1
            self.lastdir = self.direction
        elif self.i == 160:
            self.direction = 0
        elif self.i == 270:
            self.direction = self.lastdir
            self.i = 0
        self.odom.twist.twist.linear.x = self.direction*self.vx
        self.cmd_vel_pub.publish(self.odom.twist.twist)
        self.i = self.i + 1


def main(args=None):
    rclpy.init(args=args)
    flip = Flip()
    rclpy.spin(flip)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
