/**************************************************************************
 * Copyright (C) 2018 RobotICan, LTD - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 **************************************************************************/
/* Author: Elhay Rauper */

#include "../include/ric_ros_observer.h"

void RicRosObserver::onUpdate(const ric::protocol::package &ric_package)
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

                toggle_pub_.publish(toggle_msg);
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

                servo_pub_.publish(servo_msg);
            }

            break;
        }
        case (int) ric::protocol::Type::BATTERY:
        {
            if (publish_battery)
            {
                ric::protocol::battery battery_pkg = (ric::protocol::battery&) ric_package;

                ric_interface_ros::Battery battery_msg;

                battery_msg.id = battery_pkg.id;
                battery_msg.value = battery_pkg.value;

                battery_pub_.publish(battery_msg);
            }

            break;
        }
    }
}

void RicRosObserver::onServoCommand(const ric_interface_ros::Servo::ConstPtr& msg)
{
    ric::protocol::servo servo_pkg;
    servo_pkg.id = msg->id;
    servo_pkg.value = msg->value;
    ric_iface_->writeCmd(servo_pkg, sizeof(ric::protocol::servo));
}

void RicRosObserver::onLedCommand(const ric_interface_ros::Led::ConstPtr& msg)
{
    ric::protocol::led led_pkg;
    led_pkg.id = msg->id;
    led_pkg.red = msg->red;
    led_pkg.green = msg->green;
    led_pkg.blue = msg->blue;
    ric_iface_->writeCmd(led_pkg, sizeof(ric::protocol::led));
}
