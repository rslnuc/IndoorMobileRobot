import rclpy
from rclpy.node import Node

from std_msgs.msg import Int16
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

class rsl_joy(Node):

    def __init__(self):
        super().__init__('rsl_joy')
        timer_period = 0.1 

        # Responsible for publishing the mapped inputs from the controller to the rover axes.
        self.vel_pub = self.create_publisher(Twist, 'rov_cmd_vel', 5)
        self.timer = self.create_timer(timer_period, self.mapped_cmd_vel_callback)

        # Publishes the state of the mobile manipulator.
        self.rover_state_enable = self.create_publisher(Bool, 'manipulator_active', 5)
        self.timer = self.create_timer(timer_period, self.manipulator_enable)

        # Publishes to determine if the deadman switch is active on the controller.
        self.deadman_pub = self.create_publisher(Bool, 'deadman', 5)
        self.timer = self.create_timer(timer_period, self.deadman)
        
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            5)
        
        # Buttons containing the axes and button states.
        self.rsl_axes = []
        self.rsl_buttons = []
        
        self.arm_toggle = False

    # Publishes the state of the deadman button.
    def deadman(self):
        msg = Bool()
        if not(self.rsl_buttons[5]):
            msg.data = True
        else:
            msg.data = False
        self.deadman_pub.publish(msg)

    # Toggles the state of the mobile manipulator (True = arm mode, False = rover mode).
    def manipulator_enable(self):
        msg = Bool()
        msg.data = self.arm_toggle
        self.rover_state_enable.publish(msg)

    # Assigns the axes and button values from joy, to the object's initialized lists.
    def joy_callback(self, msg):
        self.rsl_axes = msg.axes
        self.rsl_buttons = msg.buttons

    # For this particular rover, it maps the controller axes to the corresponding degrees of freedom.
    def mapped_cmd_vel_callback(self):
        mapped_joy = Twist()
        mapped_joy.linear.x = self.rsl_axes[1]
        mapped_joy.linear.y = self.rsl_axes[0]
        mapped_joy.angular.z = self.rsl_axes[3]
        self.vel_pub.publish(mapped_joy)


def main(args=None):
    rclpy.init(args=args)

    rsl_custom_joy = rsl_joy()

    rclpy.spin(rsl_custom_joy)

    rsl_custom_joy.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()