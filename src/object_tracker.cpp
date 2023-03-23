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

#include <ros/ros.h>
#include <ros/package.h>

#include <unordered_map>
#include <stdint.h>

#include <mrs_msgs/RangeWithCovarianceArrayStamped.h>
#include <mrs_msgs/PoseWithCovarianceArrayStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <mrs_lib/transformer.h>
#include <mrs_lib/param_loader.h>

#include "tracker.h"
#include "helperfun.h"

#define DEFAULT_OUTPUT_FRAMERATE 20.0
#define DECAY_AGE_NORMAL 3.0
#define DECAY_AGE_UNVALIDATED 1.0
#define MIN_MEASUREMENTS_TO_VALIDATION 10
#define POS_THRESH 2.0
#define MAH_THRESH 2.0
#define YAW_THRESH 1.5
#define MATCH_LEVEL_THRESHOLD_ASSOCIATE 0.3
#define MATCH_LEVEL_THRESHOLD_REMOVE 0.5

std::unordered_map<uint64_t, std::shared_ptr<Tracker>> tracker_map;
std::unique_ptr<mrs_lib::Transformer> transformer;

ros::Publisher publish_pose;
std::string output_frame;

int kalman_pose_model;
int kalman_rotation_model;
double spectral_density_pose;
double spectral_density_rotation;

void publishStates()
{
    mrs_msgs::PoseWithCovarianceArrayStamped msg;
    mrs_msgs::PoseWithCovarianceArrayStamped msg_tent;
    msg.header.frame_id = output_frame;
    msg.header.stamp = ros::Time::now();
    msg_tent.header = msg.header;

    for (auto element : tracker_map)
    {
        mrs_msgs::PoseWithCovarianceIdentified pose;
        Eigen::Quaterniond quaternion;

        auto id = element.first;
        auto tracker = element.second;

        if (tracker->get_update_count() < MIN_MEASUREMENTS_TO_VALIDATION)
            continue;

        auto state = tracker->get_state();
        auto x = state.first;
        auto P_full = state.second;

        Eigen::Matrix<double, 6, 6> P = Eigen::Matrix<double, 6, 6>::Zero();

        P(0, 0) = P_full((int)STATE::X, (int)STATE::X);
        P(0, 1) = P_full((int)STATE::X, (int)STATE::Y);
        P(0, 2) = P_full((int)STATE::X, (int)STATE::Z);

        P(1, 0) = P_full((int)STATE::Y, (int)STATE::X);
        P(1, 1) = P_full((int)STATE::Y, (int)STATE::Y);
        P(1, 2) = P_full((int)STATE::Y, (int)STATE::Z);

        P(2, 0) = P_full((int)STATE::Z, (int)STATE::X);
        P(2, 1) = P_full((int)STATE::Z, (int)STATE::Y);
        P(2, 2) = P_full((int)STATE::Z, (int)STATE::Z);

        P(3, 3) = P_full((int)STATE::ROLL, (int)STATE::ROLL);
        P(3, 4) = P_full((int)STATE::ROLL, (int)STATE::PITCH);
        P(3, 5) = P_full((int)STATE::ROLL, (int)STATE::YAW);

        P(4, 3) = P_full((int)STATE::PITCH, (int)STATE::ROLL);
        P(4, 4) = P_full((int)STATE::PITCH, (int)STATE::PITCH);
        P(4, 5) = P_full((int)STATE::PITCH, (int)STATE::YAW);

        P(5, 3) = P_full((int)STATE::YAW, (int)STATE::ROLL);
        P(5, 4) = P_full((int)STATE::YAW, (int)STATE::PITCH);
        P(5, 5) = P_full((int)STATE::YAW, (int)STATE::YAW);

        pose.id = id;
        pose.pose.position.x = x[(int)STATE::X];
        pose.pose.position.y = x[(int)STATE::Y];
        pose.pose.position.z = x[(int)STATE::Z];

        quaternion = Eigen::AngleAxisd(x[(int)STATE::ROLL], Eigen::Vector3d::UnitX()) *
                     Eigen::AngleAxisd(x[(int)STATE::PITCH], Eigen::Vector3d::UnitY()) *
                     Eigen::AngleAxisd(x[(int)STATE::YAW], Eigen::Vector3d::UnitZ());

        pose.pose.orientation.x = quaternion.x();
        pose.pose.orientation.y = quaternion.y();
        pose.pose.orientation.z = quaternion.z();
        pose.pose.orientation.w = quaternion.w();

        pose.covariance = eigenCovarianceToRos(P);
        msg.poses.push_back(pose);
    }

    publish_pose.publish(msg);
    return;
}

void update_trackers()
{
    for (auto it = tracker_map.cbegin(); it != tracker_map.cend();)
    {
        auto id = it->first;
        auto tracker = it->second;
        double age = (ros::Time::now() - tracker->get_last_correction()).toSec();

        double decay_age = DECAY_AGE_UNVALIDATED;
        if (tracker->get_update_count() > MIN_MEASUREMENTS_TO_VALIDATION)
            decay_age = DECAY_AGE_NORMAL;

        if (age > decay_age)
        {
            ROS_WARN("Deleting node 0x%X", id);
            tracker_map.erase(it++);
            continue;
        }

        tracker->predict(ros::Time::now());
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

    std::optional<geometry_msgs::TransformStamped> transformation;

    if (msg.header.frame_id != output_frame)
    {
        transformation = transformer->getTransform(msg.header.frame_id, output_frame, msg.header.stamp);

        if (not transformation)
        {
            ROS_WARN("[OBJECT TRACKER] Not found any transformation");
            return;
        }
    }

    for (auto const &measurement : msg.poses)
    {
        ROS_INFO("[OBJECT TRACKER] Fusing measurment with ID 0x%X", measurement.id);

        // convert original msg to stamped pose
        auto pose_stamped = poseIdentifiedToPoseStamped(measurement);
        pose_stamped.header = msg.header;

        if (msg.header.frame_id != output_frame)
        {
            // transform coordinates from camera to target frame
            std::optional<geometry_msgs::PoseWithCovarianceStamped> pose_transformed;
            pose_transformed = transformer->transform(pose_stamped, transformation.value());
            if (not pose_transformed)
                continue;

            pose_stamped = pose_transformed.value();
        }

        // Get data to kalman-friendly format
        auto pose_vector = poseToVector(pose_stamped);
        auto covariance = rosCovarianceToEigen(pose_stamped.pose.covariance);

        if (covariance.array().isNaN().any() or pose_vector.array().isNaN().any())
            continue;

        covariance = covarianceSanity(covariance);

        // create new Tracker instace if not available yet
        if (not tracker_map.count(measurement.id))
        {
            ROS_INFO("[OBJECT TRACKER] Creating new tracker for object ID: 0x%X", measurement.id);
            tracker_map[measurement.id] = std::make_shared<Tracker>(pose_vector,
                                                                    covariance,
                                                                    kalman_pose_model,
                                                                    kalman_rotation_model,
                                                                    spectral_density_pose,
                                                                    spectral_density_rotation);
        }

        auto tracker = tracker_map[measurement.id];

        tracker->predict(stamp);
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

    for (auto const &measurement : msg.ranges)
    {
        if (not tracker_map.count(measurement.id))
            continue;

        ROS_INFO("[OBJECT TRACKER] Got measurement for ID 0x%X: %.2f m", measurement.id, measurement.range.range);

        auto tracker = tracker_map[measurement.id];

        if (tracker->get_update_count() < MIN_MEASUREMENTS_TO_VALIDATION)
            continue;

        kalman::range_ukf_t::z_t z(measurement.range.range);
        kalman::range_ukf_t::R_t R(measurement.variance);

        tracker->predict(stamp);
        tracker->correctRange(stamp, z, R);
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

    transformer = std::make_unique<mrs_lib::Transformer>("OBJECT TRACKER", ros::Duration(0.5));
    mrs_lib::ParamLoader param_loader(nh, "Object tracker");

    param_loader.loadParam("uav_name", uav_name);
    param_loader.loadParam("output_frame", output_frame, std::string("local_origin"));
    param_loader.loadParam("output_framerate", output_framerate, double(DEFAULT_OUTPUT_FRAMERATE));

    param_loader.loadParam("kalman_pose_model", kalman_pose_model);
    param_loader.loadParam("kalman_rotation_model", kalman_rotation_model);
    param_loader.loadParam("spectral_density_pose", spectral_density_pose);
    param_loader.loadParam("spectral_density_rotation", spectral_density_rotation);

    transformer->setDefaultPrefix(uav_name);

    ros::Subscriber pose_sub = nh.subscribe("poses", 100, pose_callback);
    ros::Subscriber range_sub = nh.subscribe("range", 100, range_callback);

    publish_pose = nh.advertise<mrs_msgs::PoseWithCovarianceArrayStamped>("filtered_poses", 10);

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
