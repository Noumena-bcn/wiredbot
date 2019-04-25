#! /usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray

from geometry_msgs.msg import Twist

ch3_data = 0
_cmd_vel_pub = rospy.Publisher('/wiredbot/cmd_vel', Twist, queue_size=3)


def move_cmd(direction="-"):
    rospy.init_node('wiredbot_teleop_x8r')
    _cmd_vel_sub = rospy.Subscriber('/wiredbot/arduino_A', Float32MultiArray, callback_wiredbot_teleop)
    _twist_object = Twist()

    rate = rospy.Rate(2)
    if direction != "stop":
        while not rospy.is_shutdown():
            _twist_object.linear.x = ch3_data
            _cmd_vel_pub.publish(_twist_object)
            rate.sleep()
    else:
        _twist_object.linear.x = 0.0
        _cmd_vel_pub.publish(_twist_object)
        rate.sleep()
    rospy.spin()


def callback_wiredbot_teleop(msg):
    """
    pins_msg.data[0] = 0.0;
    pins_msg.data[1] = 0.0;
    pins_msg.data[2] = (int)ch_3;
    pins_msg.data[3] = 0.0;
    pins_msg.data[4] = 0.0;
    pins_msg.data[5] = 0.0;
    pins_msg.data[6] = 0.0;
    pins_msg.data[7] = 0.0;
    pins_msg.data[8] = 0.0;
    pins_msg.data[9] = 0.0;
    pins_msg.data[10] = 0.0;
    """
    forward_dir = (msg.data[2] > PWM_FORWARD_MIN) and (msg.data[2] < PWM_FORWARD_MAX)
    backward_dir = (msg.data[2] > PWM_BACKWARDS_MAX) and (msg.data[2] < PWM_BACKWARDS_MIN)

    global ch3_data
    if forward_dir:
        pwm_value = map(msg.data[2], PWM_FORWARD_MIN, PWM_FORWARD_MAX, CMD_NEUTRAL, CMD_VEL_MAX_FORWARD, "FORWARD")
    elif backward_dir:
        pwm_value = map(msg.data[2], PWM_BACKWARDS_MIN, PWM_BACKWARDS_MAX, CMD_NEUTRAL, CMD_VEL_MAX_BACKWARDS,
                        "BACKWARDS")
    else:
        pwm_value = map(CMD_NEUTRAL, 0, 0, 0, 0)

    ch3_data = int(pwm_value)


def map(_input, in_min, in_max, out_min, out_max, direction="NEUTRAL"):
    if _input != 0:
        output = (_input - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    else:
        output = 0
    rospy.logdebug("REMAP: %s: input:%f output: %f in_min: :%f in_max:%f out_min:%f out_max:%f", direction, _input,
                   output, in_min, in_max, out_min, out_max)
    return output


if __name__ == '__main__':
    try:
        PWM_BACKWARDS_MIN = rospy.get_param("/wiredbot/wiredbot_teleop_x8r/PWM_BACKWARDS_MIN")
        PWM_BACKWARDS_MAX = rospy.get_param("/wiredbot/wiredbot_teleop_x8r/PWM_BACKWARDS_MAX")
        PWM_NEUTRAL_MIN = rospy.get_param("/wiredbot/wiredbot_teleop_x8r/PWM_NEUTRAL_MIN")
        PWM_NEUTRAL_MAX = rospy.get_param("/wiredbot/wiredbot_teleop_x8r/PWM_NEUTRAL_MAX")
        PWM_FORWARD_MIN = rospy.get_param("/wiredbot/wiredbot_teleop_x8r/PWM_FORWARD_MIN")
        PWM_FORWARD_MAX = rospy.get_param("/wiredbot/wiredbot_teleop_x8r/PWM_FORWARD_MAX")
        CMD_NEUTRAL = 0
        CMD_VEL_MAX_FORWARD = 128
        CMD_VEL_MAX_BACKWARDS = 112

        move_cmd()
    except rospy.ROSInterruptException:
        move_cmd('stop')
        pass
