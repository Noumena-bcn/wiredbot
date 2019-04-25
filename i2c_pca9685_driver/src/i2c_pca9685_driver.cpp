//
// Created by starsky on 8/04/19.
//

#include <boost/regex.hpp>
#include <boost/assign.hpp>
#include <boost/algorithm/cxx11/any_of.hpp>
#include <string>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <sstream>
#include <i2c_pca9685_driver/i2c_pca9685_driver.h>
#include <i2c_pca9685_driver/PWMValues.h>

namespace i2c_pca9685_driver {

    using boost::assign::list_of;
    using boost::algorithm::any_of_equal;

    PCA9685_driver::PCA9685_driver(ros::NodeHandle &nh, ros::NodeHandle &private_nh) {
        // PCA9685 ---> Init PWM channel values
        // PCA9685 ---> Set the logger level to debug from a parameter, e.g. settable from launch
        bool set_log_debug(false);

        if (private_nh.getParam("set_logger_level_debug", set_log_debug) && set_log_debug) {
            if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
                ros::console::notifyLoggerLevelsChanged();
            }
        }

        ROS_DEBUG("Initializing i2c_pca9685_driver::PCA9685_driver");

        // PCA9685 ---> Set configuration for the type of ROBOT_TYPE
        if (!private_nh.getParam("robot_type", ROBOT_TYPE) || ROBOT_TYPE.empty()) {
            ROS_FATAL("i2c_pca9685_driver::PCA9685_driver-> Parameter 'robot_type' is required.");
            ros::shutdown();
            return;
        } else {
            // PCA9685 ---> Set configuration for the type of DEVICE
            if (!private_nh.getParam("device", DEVICE) || DEVICE.empty()) {
                ROS_FATAL("i2c_pca9685_driver::PCA9685_driver-> Parameter 'device' is required.");
                ros::shutdown();
                return;
            } else {
                // PCA9685 ---> Check configuration for the type of DEVICE
                // PCA9685 ---> Set configuration for the JETSON_TX2
                if (DEVICE == JETSON_TX2) {
                    private_nh.getParam("BUS_TX2_DEFAULT", BUS);
                    private_nh.getParam("ADDRESS", ADDRESS);
                } else {
                    ROS_FATAL("i2c_pca9685_driver::PCA9685_driver-> Parameter 'device = %s' is invalid.",
                              DEVICE.c_str());
                    ros::shutdown();
                    return;
                }
            }
            // PCA9685 ---> Check configuration for the type of MOTOR_DRIVER
            if (!private_nh.getParam("motor_driver", MOTOR_DRIVER) || MOTOR_DRIVER.empty()) {
                ROS_FATAL("i2c_pca9685_driver::PCA9685_driver-> Parameter 'motor_driver' is required.");
                ros::shutdown();
                return;
            } else {
                // PCA9685 ---> Check configuration for the type of MOTOR_DRIVER
                // PCA9685 ---> Set configuration for the type of MOTOR_DRIVER SABERTOOTH_2X60
                if (MOTOR_DRIVER == SABERTOOTH_2X60) {

                    if (!private_nh.getParam("PERIOD_HZ", PERIOD_HZ)) {
                        ROS_FATAL("{ PCA9685_driver: { parameter: { name: 'PERIOD_HZ', value: NONE'}}}");
                        ros::shutdown();
                        return;
                    }
                    if (!private_nh.getParam("MOTOR_A_CHANNEL", MOTOR_A_CHANNEL)) {
                        ROS_FATAL("{ PCA9685_driver: { parameter: { name: 'MOTOR_A_CHANNEL', value: NONE'}}}");
                        ros::shutdown();
                        return;
                    }
                    if (!private_nh.getParam("MOTOR_B_CHANNEL", MOTOR_B_CHANNEL)) {
                        ROS_FATAL("{ PCA9685_driver: { parameter: { name: 'MOTOR_B_CHANNEL', value: NONE'}}}");
                        ros::shutdown();
                        return;
                    }
                    if (!private_nh.getParam("PWM_BACKWARDS_MIN", PWM_BACKWARDS_MIN)) {
                        ROS_FATAL("{ PCA9685_driver: { parameter: { name: 'PWM_BACKWARDS_MIN', value: NONE'}}}");
                        ros::shutdown();
                        return;
                    }
                    if (!private_nh.getParam("PWM_BACKWARDS_MAX", PWM_BACKWARDS_MAX)) {
                        ROS_FATAL("{ PCA9685_driver: { parameter: { name: 'PWM_BACKWARDS_MAX', value: NONE'}}}");
                        ros::shutdown();
                        return;
                    }
                    if (!private_nh.getParam("PWM_NEUTRAL_MIN", PWM_NEUTRAL_MIN)) {
                        ROS_FATAL("{ PCA9685_driver: { parameter: { name: 'PWM_NEUTRAL_MIN', value: NONE'}}}");
                        ros::shutdown();
                        return;
                    }
                    if (!private_nh.getParam("PWM_NEUTRAL_MAX", PWM_NEUTRAL_MAX)) {
                        ROS_FATAL("{ PCA9685_driver: { parameter: { name: 'PWM_NEUTRAL_MAX', value: NONE'}}}");
                        ros::shutdown();
                        return;
                    }
                    if (!private_nh.getParam("PWM_FORWARD_MIN", PWM_FORWARD_MIN)) {
                        ROS_FATAL("{ PCA9685_driver: { parameter: { name: 'PWM_FORWARD_MIN', value: NONE'}}}");
                        ros::shutdown();
                        return;
                    }
                    if (!private_nh.getParam("PWM_FORWARD_MAX", PWM_FORWARD_MAX)) {
                        ROS_FATAL("{ PCA9685_driver: { parameter: { name: 'PWM_FORWARD_MAX', value: NONE'}}}");
                        ros::shutdown();
                        return;
                    }
                    // PCA9685 ---> Set configuration for CMD_NEUTRAL CMD_VEL_MAX_FORWARD CMD_VEL_MAX_BACKWARDS
                    PCA9685_driver::SetCMDVelLimits();
                } else {
                    ROS_FATAL("i2c_pca9685_driver::PCA9685_driver-> Parameter 'motor_driver = %s' is invalid.",
                              MOTOR_DRIVER.c_str());
                    ros::shutdown();
                    return;
                }
            }
        }
    }

    PCA9685_driver::~PCA9685_driver() {
        ROS_DEBUG("Destructing i2c_pca9685_driver::PCA9685_driver");
    }

    bool PCA9685_driver::init() {
        // PCA9685 ---> ROS, Debugging the driver
        PCA9685_driver::ROSInitDebug();
        return true;
    }

    int PCA9685_driver::main() {
        if (ROBOT_TYPE == WIREDBOT) {
            this->_cmd_vel_sub = _nh.subscribe("/wiredbot/cmd_vel", 5, &PCA9685_driver::CMDVelCallback, this);
        } else {
            this->_PWMPulsesSub = _nh.subscribe("/i2c_pca9685_driver/PCA9685", 5, &PCA9685_driver::PWMPulsesCallback,
                                                this);
        }
        ros::spin();
        return 0;
    }

    void PCA9685_driver::PWMPulsesCallback(const i2c_pca9685_driver::PWMValues::ConstPtr &msg) {
        // PCA9685 ---> ROS, Callback message

        if (ROBOT_TYPE == WIREDBOT) {
            ROS_DEBUG("PWM_MOTOR_A: %i", PWM_BACKWARDS_MIN);
            MOTOR_A_CHANNEL = msg->ch0;
        }

        // PCA9685 ---> ROS, Callback message
        ROS_DEBUG("{ PWM_A: { ch0:  %i, ch1:  %i, ch2:  %i, ch3:  %i}}", msg->ch0, msg->ch1, msg->ch2, msg->ch3);
        ROS_DEBUG("{ PWM_B: { ch4:  %i, ch5:  %i, ch6:  %i, ch7:  %i}}", msg->ch4, msg->ch5, msg->ch6, msg->ch7);
        ROS_DEBUG("{ PWM_C: { ch8:  %i, ch9:  %i, ch10: %i, ch11: %i}}", msg->ch8, msg->ch9, msg->ch10, msg->ch11);
        ROS_DEBUG("{ PWM_D: { ch12: %i, ch13: %i, ch14: %i, ch15: %i}}", msg->ch12, msg->ch13, msg->ch14, msg->ch15);
    }

    void PCA9685_driver::CMDVelCallback(const geometry_msgs::Twist::ConstPtr &vel_msg) {
        if (ROBOT_TYPE == WIREDBOT) {
            MOTOR_A_CHANNEL = this->MotorSpeed2PWM(vel_msg->linear.x);
            ROS_DEBUG("PWM_MOTOR_A: %i", MOTOR_A_CHANNEL);
        }
    }

    void PCA9685_driver::ROSInitDebug() {
        // PCA9685 ---> ROS, Debugging the driver by getting the log of the parameters
        ROS_DEBUG("{ PCA9685_driver: { parameter: { name: 'ROBOT_TYPE', value: '%s'}}}", this->ROBOT_TYPE.c_str());
        ROS_DEBUG("{ PCA9685_driver: { parameter: { name: 'DEVICE', value: '%s'}}}", this->DEVICE.c_str());
        ROS_DEBUG("{ PCA9685_driver: { parameter: { name: 'MOTOR_DRIVER', value: '%s'}}}", this->MOTOR_DRIVER.c_str());
        ROS_DEBUG("{ PCA9685_driver: { parameter: { name: 'PERIOD_HZ', value: '%d'}}}", this->PERIOD_HZ);
        ROS_DEBUG("{ PCA9685_driver: { parameter: { name: 'MOTOR_A_CHANNEL', value: '%d'}}}", this->MOTOR_A_CHANNEL);
        ROS_DEBUG("{ PCA9685_driver: { parameter: { name: 'MOTOR_B_CHANNEL', value: '%d'}}}", this->MOTOR_B_CHANNEL);
        ROS_DEBUG("{ PCA9685_driver: { parameter: { name: 'PWM_BACKWARDS_MIN', value: '%d'}}}", this->PWM_BACKWARDS_MIN);
        ROS_DEBUG("{ PCA9685_driver: { parameter: { name: 'PWM_BACKWARDS_MAX', value: '%d'}}}", this->PWM_BACKWARDS_MAX);
        ROS_DEBUG("{ PCA9685_driver: { parameter: { name: 'PWM_NEUTRAL_MIN', value: '%d'}}}", this->PWM_NEUTRAL_MIN);
        ROS_DEBUG("{ PCA9685_driver: { parameter: { name: 'PWM_NEUTRAL_MAX', value: '%d'}}}", this->PWM_NEUTRAL_MAX);
        ROS_DEBUG("{ PCA9685_driver: { parameter: { name: 'PWM_FORWARD_MIN', value: '%d'}}}", this->PWM_FORWARD_MIN);
        ROS_DEBUG("{ PCA9685_driver: { parameter: { name: 'PWM_FORWARD_MAX', value: '%d'}}}", this->PWM_FORWARD_MAX);
        ROS_DEBUG("{ PCA9685_driver: { parameter: { name: 'CMD_VEL_MAX_FORWARD', value: '%d'}}}", this->CMD_VEL_MAX_FORWARD);
        ROS_DEBUG("{ PCA9685_driver: { parameter: { name: 'CMD_NEUTRAL', value: '%d'}}}", this->CMD_NEUTRAL);
        ROS_DEBUG("{ PCA9685_driver: { parameter: { name: 'CMD_VEL_MAX_BACKWARDS', value: '%d'}}}", this->CMD_VEL_MAX_BACKWARDS);
    }

    float PCA9685_driver::map(float input, float in_min, float in_max, float out_min, float out_max) {
        float output;
        if(input != 0){
            output = (input - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
        } else {
            output = PWM_NEUTRAL_MAX - 50;
        }
        ROS_DEBUG("REMAP: input:%f output: %f in_min: :%f in_max:%f out_min:%f out_max:%f", input, output, in_min, in_max, out_min, out_max);
        return output;
    }

    float PCA9685_driver::MotorSpeed2PWM(float linear_AxisValue) {
        // PCA9685 ---> FORWARD PWM VALUES
        if (linear_AxisValue > CMD_NEUTRAL) {
            // PCA9685 ---> Set the FORWARD PWM VALUES to the limit max boundary
            if(linear_AxisValue > CMD_VEL_MAX_FORWARD){
                linear_AxisValue = CMD_VEL_MAX_FORWARD;
            }
            // PCA9685 ---> GET the FORWARD PWM VALUE
            PWM_INPUT = static_cast<int16_t>(this->map(linear_AxisValue, CMD_NEUTRAL, CMD_VEL_MAX_FORWARD, PWM_FORWARD_MIN, PWM_FORWARD_MAX));
        } else if (linear_AxisValue < CMD_NEUTRAL) {
            // PCA9685 ---> Set the BACKWARDS PWM VALUES to the limit min boundary
            if(linear_AxisValue < CMD_VEL_MAX_BACKWARDS){
                linear_AxisValue = CMD_VEL_MAX_BACKWARDS;
            }
            // PCA9685 ---> GET the BACKWARDS PWM VALUE
            PWM_INPUT = static_cast<int16_t>(this->map(-linear_AxisValue, CMD_NEUTRAL, -CMD_VEL_MAX_BACKWARDS, PWM_BACKWARDS_MIN, PWM_BACKWARDS_MAX));
        } else {
            // PCA9685 ---> Set the NEUTRAL PWM VALUES by setting the max PWM_NEUTRAL_MAX and subtract 50 units
            PWM_INPUT = static_cast<int16_t>(this->map(linear_AxisValue, 0.0, 0.0, 0.0, 0.0));
        }
        return PWM_INPUT;
    }

    void PCA9685_driver::SetCMDVelLimits(){
        // PCA9685 ---> Set configuration for CMD_NEUTRAL CMD_VEL_MAX_FORWARD CMD_VEL_MAX_BACKWARDS
        CMD_NEUTRAL = 0;
        CMD_VEL_MAX_FORWARD = PWM_FORWARD_MAX - PWM_FORWARD_MIN;
        CMD_VEL_MAX_BACKWARDS = -(PWM_BACKWARDS_MAX - PWM_BACKWARDS_MIN);
        ROS_DEBUG("{ CMD_NEUTRAL: %i, CMD_VEL_MAX_FORWARD:  %i, CMD_VEL_MAX_BACKWARDS:  %i}", CMD_NEUTRAL, CMD_VEL_MAX_FORWARD, CMD_VEL_MAX_BACKWARDS);
    }
}