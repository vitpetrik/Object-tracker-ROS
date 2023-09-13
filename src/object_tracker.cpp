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
#include <std_msgs/String.h>

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
std::string distance_frame;

int kalman_pose_model;
int kalman_rotation_model;
double spectral_density_pose;
double spectral_density_rotation;
double time_to_live;

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
            ROS_WARN("Deleting node 0x%X", id);
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
    if (msg.poses.empty())
        return;
    ROS_DEBUG("[OBJECT TRACKER] Getting %ld pose measurements", msg.poses.size());

    ros::Time stamp = msg.header.stamp;
    ros::Duration age = ros::Time::now() - stamp;

    ROS_INFO("[OBJECT TRACKER] Pose message from %.3f", stamp.toSec());

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
            ROS_INFO_THROTTLE(0.5, "[OBJECT TRACKER] Creating new tracker for object ID: 0x%lX", measurement.id);
            tracker_map[measurement.id] = std::make_shared<Tracker>(stamp,
                                                                    pose_vector,
                                                                    covariance,
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
            continue;
        }

        auto tracker = tracker_map[measurement.id];
        tracker->correctPose(stamp, pose_vector, covariance);
    }
    return;
}

void range_callback(const mrs_msgs::RangeWithCovarianceArrayStamped &msg)
{
    if (msg.ranges.empty())
        return;
    ROS_DEBUG("[OBJECT TRACKER] Getting %ld range measurements", msg.ranges.size());

    ros::Time stamp = msg.header.stamp;

    ROS_INFO("[OBJECT TRACKER] Range message from %.3f", stamp.toSec());

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
                ROS_INFO("[OBJECT TRACKER] Probably tranform is not yet available");
                transformation = transformer->getTransform(kalman_frame, msg.header.frame_id, ros::Time(0));
            }
            if (not transformation)
            {
                ROS_WARN("[OBJECT TRACKER] Not found any transformation, exiting");
                return;
            }
            else
            {
                ROS_INFO("[OBJECT TRACKER] Found transformation for ros::Time(0)");
            }
        }
    }

    for (auto const &measurement : msg.ranges)
    {
        if (not tracker_map.count(measurement.id))
            continue;

        ROS_INFO_THROTTLE(0.5, "[OBJECT TRACKER] Distance measurement for ID 0x%lX: %.2f m", measurement.id, measurement.range.range);

        auto tracker = tracker_map[measurement.id];

        if (tracker->get_pose_count() < MIN_MEASUREMENTS_TO_VALIDATION)
        {
            ROS_WARN("[OBJECT TRACKER] Not enough pose measurements for tracker 0x%lX", measurement.id);
            continue;
        }

        kalman::range_ukf_t::z_t z(measurement.range.range);
        kalman::range_ukf_t::R_t R(measurement.variance);

        tracker->correctRange(stamp, z, R, transformation.value());
    }

    return;
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
    param_loader.loadParam("kalman_frame", kalman_frame, std::string("local_origin"));
    param_loader.loadParam("output_framerate", output_framerate, double(DEFAULT_OUTPUT_FRAMERATE));

    param_loader.loadParam("kalman_pose_model", kalman_pose_model);
    param_loader.loadParam("kalman_rotation_model", kalman_rotation_model);
    param_loader.loadParam("spectral_density_pose", spectral_density_pose);
    param_loader.loadParam("spectral_density_rotation", spectral_density_rotation);
    param_loader.loadParam("time_to_live", time_to_live);

    transformer->setDefaultPrefix("");

    ros::Subscriber pose_sub = nh.subscribe("poses", 10, pose_callback);
    ros::Subscriber utm_sub = nh.subscribe("utm", 10, pose_callback);
    ros::Subscriber range_sub = nh.subscribe("range", 10, range_callback);

    publish_pose = nh.advertise<mrs_msgs::PoseWithCovarianceArrayStamped>("filtered_poses", 10);
    uav_status = nh.advertise<std_msgs::String>("uav_status", 1);

    ros::Rate publish_rate(output_framerate);

    while (ros::ok())
    {
        update_trackers();
        publishStates();

        ros::spinOnce();
        publish_rate.sleep();
    }

    ros::spin();

    return 0;
}
