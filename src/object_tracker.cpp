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

#include "tracker.h"
#include "helperfun.h"

std::string output_frame = "local_origin";

std::unordered_map<uint64_t, Tracker> tracker_map;
std::unique_ptr<mrs_lib::Transformer> transformer;

void pose_callback(const mrs_msgs::PoseWithCovarianceArrayStamped &msg)
{
    if (msg.poses.empty())
        return;
    ROS_DEBUG("[OBJECT_TRACKER] Getting %ld pose measurements", msg.poses.size());

    ros::Time stamp = msg.header.stamp;

    std::optional<geometry_msgs::TransformStamped> transformation;
    transformation = transformer->getTransform(msg.header.frame_id, output_frame, msg.header.stamp);

    if (not transformation)
    {
        ROS_WARN("[OBJECT_TRACKER] Not found any transformation");
        return;
    }

    for (auto const &measurement : msg.poses)
    {
        // convert original msg to stamped pose
        auto pose_stamped = poseIdentifiedToPoseStamped(measurement);
        pose_stamped.header = msg.header;

        // transform coordinates to target frame
        std::optional<geometry_msgs::PoseWithCovarianceStamped> pose_transformed;
        pose_transformed = transformer->transform(pose_stamped, transformation.value());
        if (not pose_transformed)
            continue;

        // Get data to kalman-friendly format
        auto pose_vector = poseToVector(pose_transformed.value());
        auto covariance = rosCovarianceToEigen(pose_transformed.value().pose.covariance);

        if (covariance.array().isNaN().any() or pose_vector.array().isNaN().any())
            continue;

        covarianceSanity(covariance);

        // create new Tracker instace if not available yet
        if (not tracker_map.count(measurement.id))
            tracker_map[measurement.id] = Tracker();

        auto &tracker = tracker_map[measurement.id];

        tracker.predict(stamp);
        tracker.correctPose(pose_vector, covariance);
    }
}

void range_callback(const mrs_msgs::RangeWithCovarianceArrayStamped &msg)
{
    if (msg.ranges.empty())
        return;
    ROS_DEBUG("[OBJECT_TRACKER] Getting %ld range measurements", msg.ranges.size());

    ros::Time stamp = msg.header.stamp;

    for (auto const &measurement : msg.ranges)
    {
        if (not tracker_map.count(measurement.id))
            continue;

        auto &tracker = tracker_map[measurement.id];
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "object_tracker");
    ros::NodeHandle nh("~");

    ROS_INFO("[OBJECT_TRACKER]: Node set");

    transformer = std::make_unique<mrs_lib::Transformer>("OBJECT TRACKER");

    ros::Subscriber pose_sub_ = nh.subscribe("~poses", 100, pose_callback);
    ros::Subscriber range_sub_ = nh.subscribe("~range", 100, range_callback);

    ros::spin();

    return 0;
}
