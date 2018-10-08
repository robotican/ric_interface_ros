/**************************************************************************
 * Copyright (C) 2018 RobotICan, LTD - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 **************************************************************************/
/* Author: Elhay Rauper */

#include <ros/ros.h>
#include <ric_interface/ric_interface.h>
#include <ric_interface_ros/Location.h>
#include <ric_interface_ros/Orientation.h>
#include <ric_interface_ros/Encoder.h>
#include <ric_interface_ros/Servo.h>
#include <ric_interface_ros/Toggle.h>
#include <ric_interface_ros/Proximity.h>
#include <ric_interface_ros/Logger.h>
#include <ric_interface_ros/Error.h>
#include <ric_interface_ros/Keepalive.h>



//TODO: open rosserver for commands to ricboard

ric::RicInterface ric_iface;


class RicRosObserver : public ric::RicObserver
{
    ros::NodeHandle* nh_;

    ros::Publisher location_pub_ ;
    ros::Publisher orientation_pub_;
    ros::Publisher encoder_pub_ ;
    ros::Publisher servo_pub_;
    ros::Publisher toggle_pub_;
    ros::Publisher proximity_pub_;
    ros::Publisher logger_pub_;
    ros::Publisher error_pub_;
    ros::Publisher ka_pub_;

    ros::Subscriber servo_cmd_sub_;

    bool publish_location = true;
    bool publish_orientation = true;
    bool publish_encoder = true;
    bool publish_servo = true;
    bool publish_toggle = true;
    bool publish_proximity = true;
    bool publish_logger = true;
    bool publish_error = true;
    bool publish_keepalive = true;


    void on_update(const ric::protocol::package &ric_package)
    {
        //fprintf(stderr, "%i", ric_package.type);
        switch (ric_package.type)
        {
            case (uint8_t) ric::protocol::Type::KEEP_ALIVE:
            {
                if (publish_keepalive)
                {
                    ric::protocol::keepalive ka_pkg = (ric::protocol::keepalive &) ric_package;

                    ric_interface_ros::Keepalive ka_msg;
                    ka_msg.id = ka_pkg.id;
                    ka_pub_.publish(ka_msg);

                    break;
                }
            }
            case (uint8_t) ric::protocol::Type::LOGGER:
            {
                if (publish_logger)
                {
                    ric::protocol::logger logger_pkg = (ric::protocol::logger &) ric_package;
                    if (logger_pkg.type == (uint8_t) ric::protocol::Type::LOGGER)
                    {
                        ric_interface_ros::Logger logger_msg;
                        logger_msg.id = logger_pkg.id;
                        logger_msg.message = logger_pkg.msg;
                        logger_pub_.publish(logger_msg);
                    }
                }


                break;
            }
            case (uint8_t) ric::protocol::Type::ERROR:
            {
                if (publish_error)
                {
                    ric::protocol::error err_pkg = (ric::protocol::error &) ric_package;
                    if (err_pkg.type == (uint8_t) ric::protocol::Type::ERROR)
                    {
                        ric_interface_ros::Error error_msg;
                        error_msg.id = err_pkg.id;
                        error_msg.code = err_pkg.code;
                        error_msg.comp_id = error_msg.comp_id;
                        error_msg.comp_type = error_msg.comp_type;
                        error_pub_.publish(error_msg);
                    }
                }


                break;
            }
            case (int) ric::protocol::Type::ORIENTATION:
            {
                if (publish_orientation)
                {
                    ric::protocol::orientation orientation_pkg = (ric::protocol::orientation &) ric_package;

                    ric_interface_ros::Orientation orientation_msg;

                    orientation_msg.id = orientation_pkg.id;
                    orientation_msg.roll = orientation_pkg.roll;
                    orientation_msg.pitch = orientation_pkg.pitch;
                    orientation_msg.yaw = orientation_pkg.yaw;
                    orientation_msg.accl_x = orientation_pkg.accl_x;
                    orientation_msg.accl_y = orientation_pkg.accl_y;
                    orientation_msg.accl_z = orientation_pkg.accl_z;
                    orientation_msg.gyro_x = orientation_pkg.gyro_x;
                    orientation_msg.gyro_y = orientation_pkg.gyro_y;
                    orientation_msg.gyro_z = orientation_pkg.gyro_z;
                    orientation_msg.mag_x = orientation_pkg.mag_x;
                    orientation_msg.mag_y = orientation_pkg.mag_y;
                    orientation_msg.mag_z = orientation_pkg.mag_z;

                    orientation_pub_.publish(orientation_msg);

                    /*fprintf(stderr, "imu:\troll: %f,\tpitch: %f,\tyaw: %f \n",
                                orientation_pkg.roll_rad * 180 / M_PI,
                                orientation_pkg.pitch_rad * 180 / M_PI,
                                orientation_pkg.yaw_rad * 180 / M_PI);
                        fprintf(stderr, "imu:\taccl_x: %f,\taccl_y: %f,\taccl_z: %f \n",
                                orientation_pkg.accl_x_rad * 180 / M_PI,
                                orientation_pkg.accl_y_rad * 180 / M_PI,
                                orientation_pkg.accl_z_rad * 180 / M_PI);
                        fprintf(stderr, "imu:\tgyro_x: %f,\tgyro_y: %f,\tgyro_z: %f \n",
                                orientation_pkg.gyro_x_rad * 180 / M_PI,
                                orientation_pkg.gyro_y_rad * 180 / M_PI,
                                orientation_pkg.gyro_z_rad * 180 / M_PI);
                        fprintf(stderr, "imu:\tmag_x: %f,\tmag_y: %f,\tmag_z: %f \n",
                                orientation_pkg.mag_x_rad * 180 / M_PI,
                                orientation_pkg.mag_y_rad * 180 / M_PI,
                                orientation_pkg.mag_z_rad * 180 / M_PI);*/
                    //TODO: PUBLISH ROS MESSAGE
                }

                break;
            }
            case (int) ric::protocol::Type::PROXIMITY:
            {
                if (publish_proximity)
                {
                    ric::protocol::proximity prox_pkg = (ric::protocol::proximity &) ric_package;
                    ric_interface_ros::Proximity prox_msg;
                    prox_msg.id = prox_pkg.id;
                    prox_msg.distance = prox_pkg.distance;
                    proximity_pub_.publish(prox_msg);
                }

                break;
            }
            case (int) ric::protocol::Type::LOCATION:
            {
                if (publish_location)
                {
                    ric::protocol::location location_pkg = (ric::protocol::location &) ric_package;

                    ric_interface_ros::Location location_msg;

                    location_msg.id = location_pkg.id;
                    location_msg.lat = location_pkg.lat;
                    location_msg.lon = location_pkg.lon;
                    location_msg.alt = location_pkg.alt;
                    location_msg.heading = location_pkg.heading;
                    location_msg.speed = location_pkg.speed;
                    location_msg.satellites = location_pkg.satellites;

                    location_pub_.publish(location_msg);

                }

                break;
            }
            case (int) ric::protocol::Type::TOGGLE:
            {
                if (publish_toggle)
                {
                    ric::protocol::toggle toggle_pkg = (ric::protocol::toggle &) ric_package;

                    ric_interface_ros::Toggle toggle_msg;

                    toggle_msg.id = toggle_pkg.id;
                    toggle_msg.on = toggle_pkg.on;

                }

                break;
            }
            case (int) ric::protocol::Type::ENCODER:
            {
                if (publish_encoder)
                {
                    ric::protocol::encoder encoder_pkg = (ric::protocol::encoder &) ric_package;

                    ric_interface_ros::Encoder encoder_msg;

                    encoder_msg.id = encoder_pkg.id;
                    encoder_msg.ticks = encoder_pkg.ticks;
                    encoder_pub_.publish(encoder_msg);
                }

                break;
            }
            case (int) ric::protocol::Type::SERVO:
            {
                if (publish_servo)
                {
                    ric::protocol::servo servo_pkg = (ric::protocol::servo&) ric_package;

                    ric_interface_ros::Servo servo_msg;

                    servo_msg.id = servo_pkg.id;
                    servo_msg.value = servo_pkg.value;

                    encoder_pub_.publish(servo_msg);
                }

                break;
            }
        }
    }


    void onServoCommand(const ric_interface_ros::Servo::ConstPtr& msg)
    {
        ric::protocol::servo servo_pkg;
        servo_pkg.id = msg->id;
        servo_pkg.value = msg->value;
        ric_iface.writeCmd((ric::protocol::package&)servo_pkg, sizeof(ric::protocol::servo));
    }

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
        logger_pub_ = nh.advertise<ric_interface_ros::Logger>("ric/logger", 10);
        error_pub_ = nh.advertise<ric_interface_ros::Error>("ric/error", 10);
        ka_pub_ = nh.advertise<ric_interface_ros::Keepalive>("ric/keepalive", 10);

        servo_cmd_sub_ = nh.subscribe("ric/servo/cmd", 10, &RicRosObserver::onServoCommand, this);

        ros::param::get("~location", publish_location);
        ros::param::get("~orientation", publish_orientation);
        ros::param::get("~encoder", publish_encoder);
        ros::param::get("~servo", publish_servo);
        ros::param::get("~toggle", publish_toggle);
        ros::param::get("~proximity", publish_proximity);
        ros::param::get("~logger", publish_logger);
        ros::param::get("~error", publish_error);
    }
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "ric_interface_node");
    ros::NodeHandle nh;

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

//        if (ros::Time::now() - prev_time >= ros::Duration(0.03)) { // 50 hz
//            ric::protocol::servo actu_pkg;
//            actu_pkg.value = 2000;
//            ric_iface.writeCmd((ric::protocol::actuator&)actu_pkg, sizeof(ric::protocol::servo));
//            //ros::Duration(0.003).sleep();
//            ric_iface.writeCmd((ric::protocol::actuator&)actu_pkg, sizeof(ric::protocol::servo));
//            //ros::Duration(0.003).sleep();
//            ric_iface.writeCmd((ric::protocol::actuator&)actu_pkg, sizeof(ric::protocol::servo));
//          //  ros::Duration(0.003).sleep();
//            ric_iface.writeCmd((ric::protocol::actuator&)actu_pkg, sizeof(ric::protocol::servo));
//           // ros::Duration(0.003).sleep();
//            ric_iface.writeCmd((ric::protocol::actuator&)actu_pkg, sizeof(ric::protocol::servo));
//         //   ros::Duration(0.003).sleep();
//
//
//            prev_time = ros::Time::now();
//        }
        ros::Duration(0.001).sleep();
        ros::spinOnce();
    }
}



