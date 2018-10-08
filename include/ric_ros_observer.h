/**************************************************************************
 * Copyright (C) 2018 RobotICan, LTD - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 **************************************************************************/
/* Author: Elhay Rauper */

#ifndef RIC_INTERFACE_ROS_RIC_ROS_OBSERVER_H
#define RIC_INTERFACE_ROS_RIC_ROS_OBSERVER_H

#include <ros/ros.h>
#include <ric_interface/ric_interface.h>
#include <ric_interface_ros/Location.h>
#include <ric_interface_ros/Orientation.h>
#include <ric_interface_ros/Encoder.h>
#include <ric_interface_ros/Servo.h>
#include <ric_interface_ros/Toggle.h>
#include <ric_interface_ros/Proximity.h>
#include <ric_interface_ros/Battery.h>
#include <ric_interface_ros/Led.h>
#include <ric_interface_ros/Logger.h>
#include <ric_interface_ros/Error.h>
#include <ric_interface_ros/Keepalive.h>

class RicRosObserver : public ric::RicObserver
{
    ros::NodeHandle* nh_;

    ros::Publisher location_pub_ ;
    ros::Publisher orientation_pub_;
    ros::Publisher encoder_pub_ ;
    ros::Publisher servo_pub_;
    ros::Publisher toggle_pub_;
    ros::Publisher proximity_pub_;
    ros::Publisher battery_pub_;
    ros::Publisher logger_pub_;
    ros::Publisher error_pub_;
    ros::Publisher ka_pub_;

    ros::Subscriber servo_cmd_sub_;
    ros::Subscriber led_cmd_sub_;

    ric::RicInterface* ric_iface_;

    bool publish_location = true;
    bool publish_orientation = true;
    bool publish_encoder = true;
    bool publish_servo = true;
    bool publish_toggle = true;
    bool publish_proximity = true;
    bool publish_battery = true;
    bool publish_logger = true;
    bool publish_error = true;
    bool publish_keepalive = true;

    RicRosObserver(ric::RicInterface& ric_iface) { ric_iface_ = &ric_iface; }

    void onUpdate(const ric::protocol::package &ric_package);

    void onServoCommand(const ric_interface_ros::Servo::ConstPtr& msg);

    void onLedCommand(const ric_interface_ros::Led::ConstPtr& msg);

public:



    RicRosObserver(ros::NodeHandle& nh)
    {
        nh_ = &nh;
        location_pub_ = nh.advertise<ric_interface_ros::Location>("ric/location", 10);
        orientation_pub_ = nh.advertise<ric_interface_ros::Orientation>("ric/orientation", 10);
        encoder_pub_ = nh.advertise<ric_interface_ros::Encoder>("ric/encoder", 10);
        servo_pub_ = nh.advertise<ric_interface_ros::Servo>("ric/servo/data", 10);
        toggle_pub_ = nh.advertise<ric_interface_ros::Toggle>("ric/toggle", 10);
        proximity_pub_ = nh.advertise<ric_interface_ros::Proximity>("ric/proximity", 10);
        battery_pub_ = nh.advertise<ric_interface_ros::Battery>("ric/battery", 10);
        logger_pub_ = nh.advertise<ric_interface_ros::Logger>("ric/logger", 10);
        error_pub_ = nh.advertise<ric_interface_ros::Error>("ric/error", 10);
        ka_pub_ = nh.advertise<ric_interface_ros::Keepalive>("ric/keepalive", 10);

        servo_cmd_sub_ = nh.subscribe("ric/servo/cmd", 10, &RicRosObserver::onServoCommand, this);
        led_cmd_sub_ = nh.subscribe("ric/led/cmd", 10, &RicRosObserver::onLedCommand, this);

        ros::param::get("~location", publish_location);
        ros::param::get("~orientation", publish_orientation);
        ros::param::get("~encoder", publish_encoder);
        ros::param::get("~servo", publish_servo);
        ros::param::get("~toggle", publish_toggle);
        ros::param::get("~proximity", publish_proximity);
        ros::param::get("~battery", publish_battery);
        ros::param::get("~logger", publish_logger);
        ros::param::get("~error", publish_error);
    }
};


#endif //RIC_INTERFACE_ROS_RIC_ROS_OBSERVER_H
