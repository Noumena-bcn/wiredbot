//
// Created by starsky on 8/04/19.
//
#include <ros/ros.h>
#include "nodelet/loader.h"
#include <i2c_pca9685_driver/i2c_pca9685_driver.h>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "i2c_pca9685_driver_node");
    nodelet::Loader nodelet;
    nodelet::M_string remap(ros::names::getRemappings());
    nodelet::V_string nargv;
    std::string nodelet_name = ros::this_node::getName();
    nodelet.load(nodelet_name, "i2c_pca9685_driver/i2c_pca9685_driver_nodelet", remap, nargv);
    ros::spin();

    return 0;
}