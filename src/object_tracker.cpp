/**
 * @file object_tracker.cpp
 * @author Vit Petrik (petrivi2@fel.cvut.cz)
 * @brief
 * @version 0.1
 * @date 2023-02-26
 *
 * @copyright Copyright (c) 2023
 *
 */

#if __INTELLISENSE__
#undef __ARM_NEON
#undef __ARM_NEON__
#endif

#include <ros/ros.h>
#include <ros/package.h>

#include <unordered_map>
#include <sstream>
#include <stdint.h>

#include <mrs_msgs/RangeWithCovarianceArrayStamped.h>
#include <mrs_msgs/PoseWithCovarianceArrayStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <mrs_msgs/NavSatFixArrayStamped.h>
#include <mrs_msgs/NavSatFixIdentified.h>
#include <sensor_msgs/NavSatFix.h>

#include <std_msgs/String.h>

#include <mrs_lib/gps_conversions.h>
#include <mrs_lib/transformer.h>
#include <mrs_lib/param_loader.h>

#include "tracker.h"
#include "helperfun.h"

#define DEFAULT_OUTPUT_FRAMERATE 20.0
#define MIN_MEASUREMENTS_TO_VALIDATION 1
#define POS_THRESH 2.0
#define MAH_THRESH 2.0
#define YAW_THRESH 1.5
#define MATCH_LEVEL_THRESHOLD_ASSOCIATE 0.3
#define MATCH_LEVEL_THRESHOLD_REMOVE 0.5

std::unordered_map<uint64_t, std::shared_ptr<Tracker>> tracker_map;
std::shared_ptr<mrs_lib::Transformer> transformer;

ros::Publisher publish_pose;
ros::Publisher uav_status;
std::string kalman_frame;
std::string gps_frame;

int kalman_pose_model;
int kalman_rotation_model;
double spectral_density_pose;
double spectral_density_rotation;
double time_to_live;

bool use_uvdar, use_uwb, use_gps;

// Variables for GPS to UTM conversion
double _utm_origin_x_, _utm_origin_y_;
int _utm_origin_units_ = 0;
double rtk_local_origin_z_ = 0.0;
double land_position_x_, land_position_y_;
bool land_position_set_ = false;
double gps_altitude_ = 0;

void publishStates()
{
    ros::Time stamp = ros::Time::now();

    mrs_msgs::PoseWithCovarianceArrayStamped msg;
    mrs_msgs::PoseWithCovarianceArrayStamped msg_tent;

    msg.header.stamp = stamp;
    msg.header.frame_id = kalman_frame;
    msg_tent.header = msg.header;

    for (auto element : tracker_map)
    {
        mrs_msgs::PoseWithCovarianceIdentified pose_identified;
        Eigen::Quaterniond quaternion;

        auto id = element.first;
        auto tracker = element.second;

        if (tracker->get_update_count() < MIN_MEASUREMENTS_TO_VALIDATION)
            continue;

        std::pair<kalman::x_t, kalman::P_t> result = tracker->predict(msg.header.stamp, false);

        geometry_msgs::PoseWithCovariance pose = tracker->get_PoseWithCovariance(result.first, result.second);

        pose_identified.id = id;
        pose_identified.pose = pose.pose;
        pose_identified.covariance = pose.covariance;

        msg.poses.push_back(pose_identified);
    }

    std_msgs::String msg_status;
    msg_status.data = std::string("-id detect_msg -g [OT] detects " + std::to_string(msg.poses.size()) + " targets").c_str();
    uav_status.publish(msg_status);

    publish_pose.publish(msg);
    return;
}

void update_trackers()
{
    if (ros::Time::now().toSec() <= time_to_live)
        return;

    ros::Time deadline = ros::Time::now() - ros::Duration(time_to_live);

    for (auto it = tracker_map.cbegin(); it != tracker_map.cend();)
    {
        auto id = it->first;
        auto tracker = it->second;

        int count = tracker->delete_old(deadline);

        if (tracker->get_update_count() == 0)
        {
            ROS_WARN("Deleting node 0x%lX", id);
            tracker_map.erase(it++);

            std::stringstream ss;
            std_msgs::String msg_status;

            ss << "-id delete_msg -r [OT] Deleting tracker "
               << "0x" << std::setfill('0') << std::setw(2) << std::right << std::hex << id;
            auto x = ss.str();
            msg_status.data = x.c_str();
            uav_status.publish(msg_status);
            continue;
        }

        it++;
    }
    return;
}

void pose_callback(const mrs_msgs::PoseWithCovarianceArrayStamped &msg)
{
    if (use_uvdar == false or msg.poses.empty())
        return;
    ROS_DEBUG("[OBJECT TRACKER] Getting %ld pose measurements", msg.poses.size());

    ros::Time stamp = msg.header.stamp;
    ros::Duration age = ros::Time::now() - stamp;

    if (age.toSec() > time_to_live)
    {
        ROS_WARN("[OBJECT TRACKER] Range message is %.3f sec old", age.toSec());
        std_msgs::String msg_status;
        msg_status.data = std::string("-id error_msg -R [OT] got old RANGE").c_str();
        uav_status.publish(msg_status);
        return;
    }

    std::optional<geometry_msgs::TransformStamped> transformation;

    if (msg.header.frame_id != kalman_frame)
    {
        transformation = transformer->getTransform(msg.header.frame_id, kalman_frame, ros::Time(0));

        if (not transformation)
        {
            ROS_WARN("[OBJECT TRACKER] Not found any transformation");
            return;
        }
    }

    for (auto const &measurement : msg.poses)
    {
        ROS_INFO_THROTTLE(0.5, "[OBJECT TRACKER] POSE measurement with ID 0x%lX", measurement.id);

        // convert original msg to stamped pose
        auto pose_stamped = poseIdentifiedToPoseStamped(measurement);
        pose_stamped.header = msg.header;

        if (msg.header.frame_id != kalman_frame)
        {
            // transform coordinates from camera to target frame
            std::optional<geometry_msgs::PoseWithCovarianceStamped> pose_transformed;
            pose_transformed = transformer->transform(pose_stamped, transformation.value());
            if (not pose_transformed)
                continue;

            pose_stamped = pose_transformed.value();
        }

        // Get data to kalman-friendly format
        auto pose_vector = poseToVector(pose_stamped.pose);
        auto covariance = rosCovarianceToEigen(pose_stamped.pose.covariance);

        if (covariance.array().isNaN().any() or pose_vector.array().isNaN().any())
            continue;

        covariance = covarianceSanity(covariance);

        // create new Tracker instace if not available yet
        if (not tracker_map.count(measurement.id))
        {
            ROS_INFO_THROTTLE(1.0, "[OBJECT TRACKER] Creating new tracker for object ID: 0x%lX", measurement.id);
            tracker_map[measurement.id] = std::make_shared<Tracker>(stamp,
                                                                    kalman_pose_model,
                                                                    kalman_rotation_model,
                                                                    spectral_density_pose,
                                                                    spectral_density_rotation,
                                                                    transformer);

            std::stringstream ss;
            std_msgs::String msg_status;

            ss << "-id create_msg -g [OT] Creating tracker "
               << "0x" << std::setfill('0') << std::setw(2) << std::right << std::hex << measurement.id;
            auto x = ss.str();
            msg_status.data = x.c_str();
            uav_status.publish(msg_status);
        }

        auto tracker = tracker_map[measurement.id];

        kalman::pose_lkf_t::z_t z(pose_vector);
        kalman::pose_lkf_t::R_t R(covariance);

        tracker->addMeasurement(stamp, z, R);
    }
    return;
}

void range_callback(const mrs_msgs::RangeWithCovarianceArrayStamped &msg)
{
    if (use_uwb == false or msg.ranges.empty())
        return;

    ROS_DEBUG("[OBJECT TRACKER] Getting %ld range measurements", msg.ranges.size());

    ros::Time stamp = msg.header.stamp;
    ros::Duration age = ros::Time::now() - stamp;

    if (age.toSec() > time_to_live)
    {
        ROS_WARN("[OBJECT TRACKER] Range message is %.3f sec old", age.toSec());
        std_msgs::String msg_status;
        msg_status.data = std::string("-id error_msg -R [OT] got old RANGE").c_str();
        uav_status.publish(msg_status);
        return;
    }

    std::optional<geometry_msgs::TransformStamped> transformation;

    if (msg.header.frame_id != kalman_frame)
    {
        try
        {
            transformation = transformer->getTransform(kalman_frame, msg.header.frame_id, stamp);
        }
        catch (std::exception &e)
        {
            ROS_WARN_STREAM("[OBJECT TRACKER] Exception: " << e.what());
        }

        if (not transformation)
        {
            if (ros::Time::now() - stamp < ros::Duration(0.1))
            {
                transformation = transformer->getTransform(kalman_frame, msg.header.frame_id, ros::Time(0));
            }
            if (not transformation)
            {
                ROS_WARN("[OBJECT TRACKER] Not found any transformation, exiting");
                return;
            }
        }
    }

    for (auto const &measurement : msg.ranges)
    {
        if (not tracker_map.count(measurement.id))
            continue;

        ROS_INFO_THROTTLE(1.0, "[OBJECT TRACKER] Distance measurement for ID 0x%lX: %.2f m", measurement.id, measurement.range.range);

        auto tracker = tracker_map[measurement.id];

        if (tracker->get_pose_count() < MIN_MEASUREMENTS_TO_VALIDATION)
        {
            ROS_WARN("[OBJECT TRACKER] Not enough pose measurements for tracker 0x%lX", measurement.id);
            continue;
        }

        kalman::range_ukf_t::z_t z(measurement.range.range);
        kalman::range_ukf_t::R_t R(measurement.variance);

        tracker->addMeasurement(stamp, z, R, transformation.value());
    }

    return;
}

void gps_cb(const mrs_msgs::NavSatFixArrayStamped &msg)
{
    if (use_gps == false or msg.nav_sat_fixes.size() == 0)
        return;

    if (isnan(_utm_origin_x_) or isnan(_utm_origin_y_))
    {
        ROS_ERROR_THROTTLE(1.0, "[OBJECT TRACKER] UTM origin not set, cannot convert GPS to UTM");
        return;
    }

    ros::Time stamp = msg.header.stamp;
    ros::Duration age = ros::Time::now() - stamp;

    if (age.toSec() > time_to_live)
    {
        ROS_WARN("[OBJECT TRACKER] GPS message is %.3f sec old", age.toSec());
        std_msgs::String msg_status;
        msg_status.data = std::string("-id error_msg -R [OT] got old RANGE").c_str();
        uav_status.publish(msg_status);
        return;
    }

    for (auto const &measurement : msg.nav_sat_fixes)
    {
        if (!std::isfinite(measurement.gps.latitude))
        {
            ROS_ERROR_THROTTLE(1.0, "[OBJECT TRACKER] NaN detected in RTK variable \"msg->latitude\"!!!");
            return;
        }

        if (!std::isfinite(measurement.gps.longitude))
        {
            ROS_ERROR_THROTTLE(1.0, "[OBJECT TRACKER] NaN detected in RTK variable \"msg->longitude\"!!!");
            return;
        }

        ROS_INFO_THROTTLE(1.0, "[OBJECT TRACKER] GPS measurement for ID 0x%lX", measurement.id);

        // create new Tracker instace if not available yet
        if (not tracker_map.count(measurement.id))
        {
            ROS_INFO_THROTTLE(1.0, "[OBJECT TRACKER] Creating new tracker for object ID: 0x%lX", measurement.id);
            tracker_map[measurement.id] = std::make_shared<Tracker>(stamp,
                                                                    kalman_pose_model,
                                                                    kalman_rotation_model,
                                                                    spectral_density_pose,
                                                                    spectral_density_rotation,
                                                                    transformer);

            std::stringstream ss;
            std_msgs::String msg_status;

            ss << "-id create_msg -g [OT] Creating tracker "
               << "0x" << std::setfill('0') << std::setw(2) << std::right << std::hex << measurement.id;
            auto x = ss.str();
            msg_status.data = x.c_str();
            uav_status.publish(msg_status);
        }

        auto tracker = tracker_map[measurement.id];

        // convert it to UTM

        double x, y;
        mrs_lib::UTM(measurement.gps.latitude, measurement.gps.longitude, &x, &y);

        if (not isnan(measurement.gps.altitude))
        {
            kalman::pose3d_lkf_t::z_t z = kalman::pose3d_lkf_t::z_t(x, y, measurement.gps.altitude);
            z -= kalman::pose3d_lkf_t::z_t(_utm_origin_x_, _utm_origin_y_, 0);

            ROS_INFO_THROTTLE(1.0, "[OBJECT TRACKER] GPS measurement for ID 0x%lX: x: %.2f, y: %.2f, z: %.2f", measurement.id, z(0), z(1), z(2));

            kalman::pose3d_lkf_t::R_t R = kalman::pose3d_lkf_t::R_t::Zero();

            R(0, 0) = measurement.gps.position_covariance[0];
            R(0, 1) = measurement.gps.position_covariance[1];
            R(0, 2) = measurement.gps.position_covariance[2];
            R(1, 1) = measurement.gps.position_covariance[4];
            R(1, 2) = measurement.gps.position_covariance[5];
            R(2, 2) = measurement.gps.position_covariance[8];

            R.triangularView<Eigen::Lower>() = R.transpose();
            tracker->addMeasurement(stamp, z, R);
        }
        else
        {
            kalman::pose2d_lkf_t::z_t z = kalman::pose2d_lkf_t::z_t(x, y);
            z -= kalman::pose2d_lkf_t::z_t(_utm_origin_x_, _utm_origin_y_);

            ROS_INFO_THROTTLE(1.0, "[OBJECT TRACKER] GPS measurement for ID 0x%lX: x: %.2f, y: %.2f", measurement.id, z(0), z(1));

            kalman::pose2d_lkf_t::R_t R = kalman::pose2d_lkf_t::R_t::Zero();

            R(0, 0) = measurement.gps.position_covariance[0];
            R(0, 1) = measurement.gps.position_covariance[1];
            R(1, 0) = measurement.gps.position_covariance[3];
            R(1, 1) = measurement.gps.position_covariance[4];

            tracker->addMeasurement(stamp, z, R);
        }
    }

    return;
}

void updateCallback(const ros::TimerEvent &event)
{
    update_trackers();
    publishStates();
}

int main(int argc, char **argv)
{
    std::string uav_name;
    double output_framerate;

    ros::init(argc, argv, "object_tracker");
    ros::NodeHandle nh("~");

    ROS_INFO("[OBJECT TRACKER]: Node set");

    transformer = std::make_shared<mrs_lib::Transformer>("OBJECT TRACKER", ros::Duration(0.1));
    mrs_lib::ParamLoader param_loader(nh, "Object tracker");

    param_loader.loadParam("uav_name", uav_name);
    param_loader.loadParam("kalman_frame", kalman_frame, uav_name + std::string("/local_origin"));
    param_loader.loadParam("gps_frame", gps_frame, uav_name + std::string("/gps_origin"));
    param_loader.loadParam("output_framerate", output_framerate, double(DEFAULT_OUTPUT_FRAMERATE));

    param_loader.loadParam("kalman_pose_model", kalman_pose_model);
    param_loader.loadParam("kalman_rotation_model", kalman_rotation_model);
    param_loader.loadParam("spectral_density_pose", spectral_density_pose);
    param_loader.loadParam("spectral_density_rotation", spectral_density_rotation);
    param_loader.loadParam("time_to_live", time_to_live);

    param_loader.loadParam("use_uvdar", use_uvdar, true);
    param_loader.loadParam("use_uwb", use_uwb, true);
    param_loader.loadParam("use_gps", use_gps, true);

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
        ROS_ERROR("[OBJECT_TRACKER]: Could not load UTM origin.");

        _utm_origin_x_ = nan("");
        _utm_origin_y_ = nan("");
    }

    transformer->setDefaultPrefix("");

    ros::Subscriber pose_sub, range_sub, rtk_gps_sub;
    pose_sub = nh.subscribe("uvdar", 10, pose_callback);
    range_sub = nh.subscribe("uwb", 10, range_callback);
    rtk_gps_sub = nh.subscribe("gps", 10, gps_cb);

    publish_pose = nh.advertise<mrs_msgs::PoseWithCovarianceArrayStamped>("filtered_poses", 10);
    uav_status = nh.advertise<std_msgs::String>("uav_status", 1);

    ros::Timer timer = nh.createTimer(ros::Duration(1/output_framerate), updateCallback);

    ros::spin();

    return 0;
}
