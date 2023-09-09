/**
 * @file transformer.cpp
 * @author Vit Petrik (petrivi2@fel.cvut.cz)
 * @brief
 * @version 0.1
 * @date 2023-03-18
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <ros/ros.h>
#include <ros/package.h>

#include <mrs_msgs/PoseWithCovarianceArrayStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <mrs_lib/transformer.h>
#include <mrs_lib/param_loader.h>

#include "helperfun.h"

std::string output_frame;
std::unique_ptr<mrs_lib::Transformer> transformer;

ros::Publisher publish_transformed;

float cache_time;

void pose_callback(const mrs_msgs::PoseWithCovarianceArrayStamped &msg)
{
    if (msg.poses.empty())
        return;

    ros::Time stamp = msg.header.stamp;

    mrs_msgs::PoseWithCovarianceArrayStamped msg_transformed;
    msg_transformed.header.frame_id = output_frame;
    msg_transformed.header.stamp = stamp;

    if (msg.header.frame_id == output_frame)
    {
        publish_transformed.publish(msg);
        return;
    }

    if(ros::Time::now() - stamp > ros::Duration(cache_time))
    {
        ROS_WARN("[TRANSFORMER] Message is too old");
        return;
    }

    std::optional<geometry_msgs::TransformStamped> transformation;
    try
    {
        transformation = transformer->getTransform(msg.header.frame_id, output_frame, stamp);
    }
    catch (std::exception& e)
    {
        ROS_WARN_STREAM("[TRANSFORMER] Exception: " << e.what());
    }

    if (not transformation)
    {
        if(ros::Time::now() - stamp < ros::Duration(0.1))
        {
            ROS_INFO("[TRANSFORMER] Probably tranform is not yet available");
            transformation = transformer->getTransform(msg.header.frame_id, output_frame, ros::Time(0));
        }
        if (not transformation)
        {
            ROS_WARN("[TRANSFORMER] Not found any transformation, exiting");
            return;
        }
        else
        {
            ROS_INFO("[TRANSFORMER] Found transformation for ros::Time(0)");
        }
    }

    for (auto const &measurement : msg.poses)
    {
        // convert original msg to stamped pose
        auto pose_stamped = poseIdentifiedToPoseStamped(measurement);
        pose_stamped.header = msg.header;

        // transform coordinates from camera to target frame
        std::optional<geometry_msgs::PoseWithCovarianceStamped> pose_transformed;
        pose_transformed = transformer->transform(pose_stamped, transformation.value());
        if (not pose_transformed)
            continue;

        mrs_msgs::PoseWithCovarianceIdentified temp;

        temp.id = measurement.id;
        temp.covariance = pose_transformed->pose.covariance;
        temp.pose = pose_transformed->pose.pose;

        msg_transformed.poses.push_back(temp);
    }

    publish_transformed.publish(msg_transformed);
    return;
}

int main(int argc, char **argv)
{
    std::string uav_name;
    ros::init(argc, argv, "transformer");
    ros::NodeHandle nh("~");

    ROS_INFO("[TRANSFORMED]: Node set");

    mrs_lib::ParamLoader param_loader(nh, "Transformer");

    param_loader.loadParam("uav_name", uav_name);
    param_loader.loadParam("output_frame", output_frame, std::string("local_origin"));

    param_loader.loadParam("cache_time", cache_time, 1.0f);
    transformer = std::make_unique<mrs_lib::Transformer>("TRANSFORMER", ros::Duration(cache_time));

    transformer->beQuiet();

    ros::Subscriber pose_sub = nh.subscribe("poses", 10, pose_callback);

    publish_transformed = nh.advertise<mrs_msgs::PoseWithCovarianceArrayStamped>("transformed_poses", 10);

    ros::spin();

    return 0;
}
