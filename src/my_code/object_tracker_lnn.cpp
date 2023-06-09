/**
 * @file object_tracker.cpp
 * @author Vit Petrik (petrivi2@fel.cvut.cz) and Lucas Nobrega 
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
std::shared_ptr<mrs_lib::Transformer> transformer;

ros::Publisher publish_pose;
std::string kalman_frame;
std::string distance_frame;

int kalman_pose_model;
int kalman_rotation_model;
double spectral_density_pose;
double spectral_density_rotation;

int output_id = 0;

void publishStates()
{
    mrs_msgs::PoseWithCovarianceArrayStamped msg;
    mrs_msgs::PoseWithCovarianceArrayStamped msg_tent;
    msg.header.frame_id = kalman_frame;
    msg.header.stamp = ros::Time::now();
    msg_tent.header = msg.header;

    for (auto element : tracker_map)
    {
        mrs_msgs::PoseWithCovarianceIdentified pose_identified;
        Eigen::Quaterniond quaternion;

        auto id = element.first;
        auto tracker = element.second;
        
        ROS_INFO_STREAM("[OB TRCKR] element id " << id << " tracker len " << tracker->get_update_count());


        if (tracker->get_update_count() < MIN_MEASUREMENTS_TO_VALIDATION)
            continue;

        geometry_msgs::PoseWithCovariance pose = tracker->get_PoseWithCovariance();

        pose_identified.id = id;
        pose_identified.pose = pose.pose;
        pose_identified.covariance = pose.covariance;

        msg.poses.push_back(pose_identified);
    }

    if(msg.poses.empty()){
        ROS_ERROR("[OBJECT TRACKER - publishStates] Pose message is empty");
    }
    else{
        ROS_INFO_STREAM("[OBJECT TRACKER - publishStates] Ready to publish the states." << msg.poses.cend()->pose.position.x <<" "
                                                                                        << msg.poses.cend()->pose.position.y << " " 
                                                                                        << msg.poses.cend()->pose.position.z);
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
            ROS_WARN_STREAM("Deleting node 0x" << id);
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
    
    int i = 0;
    
    //mrs_msgs::PoseWithCovarianceArrayStamped msg_cleaned;

    
    // The beginning is the same.
    //msg_cleaned.header.frame_id = msg.header.frame_id;
    //msg_cleaned.header.stamp = msg.header.stamp;
    //msg_cleaned.header.seq = msg.header.seq;

    // Poses will be calculated now, based on ID 
    for (auto& local_msg : msg.poses){
      i++;
      ROS_INFO_STREAM("[OBJECT TRACKER LNN CODE] " << i << " Got stamp: " << msg.header.stamp);
      ROS_INFO_STREAM("[OBJECT TRACKER LNN CODE] " << i << " Got pose for ID: " << local_msg.id );
      ROS_INFO_STREAM("[OBJECT TRACKER LNN CODE] " << i << " Got local_msg.id: " << local_msg.id);
      ROS_INFO_STREAM("[OBJECT TRACKER LNN CODE] " << i << " For frame: " << msg.header.frame_id <<
                                                                " got pose (x, y, z): " << local_msg.pose.position.x << " | "
                                                                                        << local_msg.pose.position.y << " | "
                                                                                        << local_msg.pose.position.z);
      
      if(local_msg.id == output_id){
        ROS_INFO_STREAM("[OBJECT TRACKER LNN CODE] Message will be inserted");
        //msg_cleaned.poses.push_back(local_msg);
        //ROS_INFO_STREAM("[OBJECT TRACKER LNN CODE] Inserted message: " << msg_cleaned.poses.size());
      }
      else{
        ROS_WARN_STREAM("[OBJECT TRACKER LNN CODE] This msg id " << local_msg.id <<" will not be processed because is waiting for the " << output_id);
      }

      ROS_INFO_STREAM("[LNN CODE] ##############################################################################");
    }

 
    //ROS_DEBUG_STREAM("[OBJECT TRACKER] Getting "<< msg_cleaned.poses.size() <<" pose measurements");
    ROS_DEBUG_STREAM("[OBJECT TRACKER] Getting "<< msg.poses.size() <<" pose measurements");

    //ros::Time stamp = msg_cleaned.header.stamp;
    ros::Time stamp = msg.header.stamp;

    std::optional<geometry_msgs::TransformStamped> transformation;

    //if (msg_cleaned.header.frame_id != kalman_frame)
    if (msg.header.frame_id != kalman_frame)
    {
        //transformation = transformer->getTransform(msg_cleaned.header.frame_id, kalman_frame, ros::Time(0));
        transformation = transformer->getTransform(msg.header.frame_id, kalman_frame, ros::Time(0));

        if (not transformation)
        {
            //ROS_WARN("[OBJECT TRACKER - pose_callback] Not found any transformation");
            //ROS_WARN_STREAM("[OBJECT TRACKER - pose_callback] Not found any transformation from " << kalman_frame << " to "<< msg_cleaned.header.frame_id);
            ROS_WARN_STREAM("[OBJECT TRACKER - pose_callback] Not found any transformation from " << kalman_frame << " to "<< msg.header.frame_id);
            return;
        }
    }

    //for (auto const &measurement : msg_cleaned.poses)
    for (auto const &measurement : msg.poses)
    {
        ROS_INFO_STREAM_THROTTLE(0.5, "[OBJECT TRACKER] Fusing measurement with ID 0x" << measurement.id);

        // convert original msg to stamped pose
        auto pose_stamped = poseIdentifiedToPoseStamped(measurement);
        //pose_stamped.header = msg_cleaned.header;
        pose_stamped.header = msg.header;

        //if (msg_cleaned.header.frame_id != kalman_frame)
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

        // create new Tracker instance if not available yet
        if (not tracker_map.count(measurement.id))
        {
            ROS_INFO_STREAM_THROTTLE(0.5, "[OBJECT TRACKER] Creating new tracker for object ID: 0x" << measurement.id);
            tracker_map[measurement.id] = std::make_shared<Tracker>(pose_vector,
                                                                    covariance,
                                                                    kalman_pose_model,
                                                                    kalman_rotation_model,
                                                                    spectral_density_pose,
                                                                    spectral_density_rotation,
                                                                    transformer);
            continue;
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

    std::optional<geometry_msgs::TransformStamped> transformation;

    if (msg.header.frame_id != kalman_frame)
    {
        transformation = transformer->getTransform(kalman_frame, msg.header.frame_id, ros::Time(0));

        if (not transformation)
        {
            ROS_WARN_STREAM("[OBJECT TRACKER - range_callback] Not found any transformation from " << kalman_frame << " to "<< msg.header.frame_id);
            return;
        }
    }

    for (auto const &measurement : msg.ranges)
    {
        if (not tracker_map.count(measurement.id))
            continue;

        ROS_INFO_STREAM_THROTTLE(0.5, "[OBJECT TRACKER] Got measurement for ID 0x" << measurement.id << ":" << measurement.range.range << "m");

        auto tracker = tracker_map[measurement.id];

        if (tracker->get_update_count() < MIN_MEASUREMENTS_TO_VALIDATION)
            continue;

        kalman::range_ukf_t::z_t z(measurement.range.range);
        kalman::range_ukf_t::R_t R(measurement.variance);

        tracker->transform(transformation.value());

        tracker->predict(stamp);
        tracker->correctRange(stamp, z, R);

        tracker->transform(transformer->inverse(transformation.value()));
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

    param_loader.loadParam("output_id", output_id);

    param_loader.loadParam("kalman_pose_model", kalman_pose_model);
    param_loader.loadParam("kalman_rotation_model", kalman_rotation_model);
    param_loader.loadParam("spectral_density_pose", spectral_density_pose);
    param_loader.loadParam("spectral_density_rotation", spectral_density_rotation);

    transformer->setDefaultPrefix("");

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
