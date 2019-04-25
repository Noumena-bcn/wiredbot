//
// Created by starsky on 8/04/19.
//
#include <iostream>
#include <boost/shared_ptr.hpp>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <i2c_pca9685_driver/i2c_pca9685_driver.h>

namespace i2c_pca9685_driver {

    class PCA9685_driver_nodelet : public nodelet::Nodelet {

    public:
        PCA9685_driver_nodelet() {}

    private:
        virtual void onInit(void);

        boost::shared_ptr<PCA9685_driver> _PCA9685_driver;

    };

    // class PCA9685_driver_nodelet
    void PCA9685_driver_nodelet::onInit() {

        ros::NodeHandle nh = this->getPrivateNodeHandle();
        // resolve node(let) name
        std::string name = nh.getUnresolvedNamespace();
        int pos = name.find_last_of('/');
        name = name.substr(pos + 1);
        char _name[name.size() + 1];
        strcpy(_name,name.c_str());

        NODELET_INFO_STREAM("Initialising nodelet... [" << name << "]");

        _PCA9685_driver.reset(new PCA9685_driver(getNodeHandle(), getPrivateNodeHandle()));

        if(_PCA9685_driver->init()){
            NODELET_INFO_STREAM("Nodelet initialised [" << name << "]");
            _PCA9685_driver->main();
        } else {
            NODELET_ERROR_STREAM("Couldn't initialise nodelet! Please restart.[" << name << "]");
        }
    }

}  // namespace i2c_pca9685_driver
PLUGINLIB_EXPORT_CLASS(i2c_pca9685_driver::PCA9685_driver_nodelet, nodelet::Nodelet);
