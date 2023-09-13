/**
 * @file lla_to_utm.cpp
 * @author Vit Petrik (petrivi2@fel.cvut.cz)
 * @brief
 * @version 0.1
 * @date 2023-09-09
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <ros/ros.h>
#include <ros/package.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <mrs_msgs/PoseWithCovarianceArrayStamped.h>
#include <mrs_msgs/NavSatFixArrayStamped.h>
#include <mrs_msgs/NavSatFixIdentified.h>
#include <sensor_msgs/NavSatFix.h>

#include <mrs_lib/transformer.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/gps_conversions.h>

std::string output_frame;
ros::Publisher publish_transformed;

double _utm_origin_x_, _utm_origin_y_;
int _utm_origin_units_ = 0;
double rtk_local_origin_z_ = 0.0;
double land_position_x_, land_position_y_;
bool land_position_set_ = false;
double gps_altitude_ = 0;

void gps_cb(const mrs_msgs::NavSatFixArrayStamped &msg)
{
    if(msg.nav_sat_fixes.size() == 0)
        return;

    ros::Time stamp = msg.header.stamp;

    mrs_msgs::PoseWithCovarianceArrayStamped msg_transformed;
    msg_transformed.header.frame_id = output_frame;
    msg_transformed.header.stamp = stamp;

    for (auto const &measurement : msg.nav_sat_fixes)
    {
        mrs_msgs::PoseWithCovarianceIdentified pose;
        pose.id = measurement.id;

        if (!std::isfinite(measurement.gps.latitude))
        {
            ROS_ERROR_THROTTLE(1.0, "[Odometry] NaN detected in RTK variable \"msg->latitude\"!!!");
            return;
        }

        if (!std::isfinite(measurement.gps.longitude))
        {
            ROS_ERROR_THROTTLE(1.0, "[Odometry] NaN detected in RTK variable \"msg->longitude\"!!!");
            return;
        }

        // convert it to UTM
        mrs_lib::UTM(measurement.gps.latitude, measurement.gps.longitude, &pose.pose.position.x, &pose.pose.position.y);
        pose.pose.position.z = 0;

        pose.pose.position.x -= _utm_origin_x_;
        pose.pose.position.y -= _utm_origin_y_;
        pose.pose.position.z -= 0;

        pose.covariance = {0};
        pose.covariance[0*6 + 0] = measurement.gps.position_covariance[0];
        pose.covariance[1*6 + 1] = measurement.gps.position_covariance[4];
        pose.covariance[2*6 + 2] = 1e6;
        pose.covariance[3*6 + 3] = 1e6;
        pose.covariance[4*6 + 4] = 1e6;
        pose.covariance[5*6 + 5] = 1e6;

        msg_transformed.poses.push_back(pose);
    }

    publish_transformed.publish(msg_transformed);
}

int main(int argc, char **argv)
{
    std::string uav_name;
    ros::init(argc, argv, "lla_to_utm");
    ros::NodeHandle nh("~");

    ROS_INFO("[LLA TO UTM]: Node set");

    mrs_lib::ParamLoader param_loader(nh, "lla_to_utm");

    param_loader.loadParam("uav_name", uav_name);
    param_loader.loadParam("output_frame", output_frame, std::string("local_origin"));

    bool is_origin_param_ok = true;
    param_loader.loadParam("utm_origin_units", _utm_origin_units_);
    if (_utm_origin_units_ == 0)
    {
        ROS_INFO("[Odometry]: Loading UTM origin in UTM units.");
        is_origin_param_ok &= param_loader.loadParam("utm_origin_x", _utm_origin_x_);
        is_origin_param_ok &= param_loader.loadParam("utm_origin_y", _utm_origin_y_);
    }
    else
    {
        double lat, lon;
        ROS_INFO("[Odometry]: Loading UTM origin in LatLon units.");
        is_origin_param_ok &= param_loader.loadParam("utm_origin_lat", lat);
        is_origin_param_ok &= param_loader.loadParam("utm_origin_lon", lon);
        mrs_lib::UTM(lat, lon, &_utm_origin_x_, &_utm_origin_y_);
        ROS_INFO("[Odometry]: Converted to UTM x: %f, y: %f.", _utm_origin_x_, _utm_origin_y_);
    }
 

    if (!is_origin_param_ok)
    {
        ROS_ERROR("[Odometry]: Could not load all mandatory parameters from world file. Please check your world file.");
        ros::shutdown();
    }

    ros::Subscriber rtk_gps_sub_ = nh.subscribe("lla_in", 1, gps_cb);
    publish_transformed = nh.advertise<mrs_msgs::PoseWithCovarianceArrayStamped>("utm_out", 10);

    ros::spin();

    return 0;
}
