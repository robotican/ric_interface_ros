/**************************************************************************
 * Copyright (C) 2018 RobotICan, LTD - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 **************************************************************************/
/* Author: Elhay Rauper */

#include <ros/ros.h>
#include <ric_interface/ric_interface.h>
#include "../include/ric_ros_observer.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ric_interface_node");
    ros::NodeHandle nh;

    ric::RicInterface ric_iface;

    std::string port = "/dev/ttyACM0";
    ros::param::get("~ric_port", port);


    ROS_INFO("[ric_interface_ros]: connecting to %s", port.c_str());
    try
    {
        ric_iface.connect(port);
    }
    catch (ric::RicException exp)
    {
        ROS_ERROR("%s", exp.what());
        exit(EXIT_FAILURE);
    }
    ROS_INFO("[ric_interface_ros]: RicBoard connected.");


    RicRosObserver ric_observer(nh);

    // register for ric messages
    ric_iface.attach(&ric_observer);

    ros::Time prev_time = ros::Time::now();

    while (ros::ok())
    {
        ric_iface.loop();

//        ric::protocol::servo servo_pkg;
//        servo_pkg.id = 30;
//        servo_pkg.value = 1800;
//        ric_iface.writeCmd(servo_pkg, sizeof(ric::protocol::servo));

        ros::Rate(100).sleep();
        ros::spinOnce();
    }
}



