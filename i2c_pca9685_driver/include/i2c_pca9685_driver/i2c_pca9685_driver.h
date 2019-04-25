//
// Created by starsky on 8/04/19.
//

#ifndef I2C_PCA9685_DRIVER_I2C_PCA9685_DRIVER_H
#define I2C_PCA9685_DRIVER_I2C_PCA9685_DRIVER_H

#include <string>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <i2c_pca9685_driver/PWMValues.h>

#define SABERTOOTH_2X60 "SABERTOOTH_2X60"
#define JETSON_TX2 "JETSON_TX2"
#define WIREDBOT "WIREDBOT"




namespace i2c_pca9685_driver {

    class PCA9685_driver {
    public:
        // constructor
        PCA9685_driver(ros::NodeHandle &nh, ros::NodeHandle &private_nh);

        // destructor
        ~PCA9685_driver();

        // driver init
        bool init();

        int main();

    private:

        // VARIABLES CLASS PCA9685_driver
        int ADDRESS;
        int BUS;
        std::string ROBOT_TYPE;
        std::string DEVICE;
        std::string MOTOR_DRIVER;

        // VARIABLE SABERTOOTH
        int PERIOD_HZ;
        int MOTOR_A_CHANNEL;
        int MOTOR_B_CHANNEL;
        int PWM_BACKWARDS_MIN;
        int PWM_BACKWARDS_MAX;
        int PWM_NEUTRAL_MIN;
        int PWM_NEUTRAL_MAX;
        int PWM_FORWARD_MIN;
        int PWM_FORWARD_MAX;
        int CMD_VEL_MAX_FORWARD;
        int CMD_VEL_MAX_BACKWARDS;
        int CMD_NEUTRAL;
        int PWM_INPUT;

        // ROS SERVICES
        std::string _name;
        ros::NodeHandle _nh;
        ros::Subscriber _PWMPulsesSub;
        ros::Subscriber _cmd_vel_sub;



        void PWMPulsesCallback(const i2c_pca9685_driver::PWMValues::ConstPtr &msg);
        void CMDVelCallback(const geometry_msgs::Twist::ConstPtr& vel_msg);
        void ROSInitDebug();
        float map(float input, float in_min, float in_max, float out_min, float out_max);
        float MotorSpeed2PWM(float cmd_vel_speed);
        void SetCMDVelLimits();
        // ROS MSG VARIABLES

    };
}

#endif //I2C_PCA9685_DRIVER_I2C_PCA9685_DRIVER_H
