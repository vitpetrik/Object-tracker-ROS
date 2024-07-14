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

#include <tf2/convert.h>

#include <unordered_map>
#include <sstream>
#include <stdint.h>

#include <mrs_msgs/RangeWithCovarianceArrayStamped.h>
#include <mrs_msgs/PoseWithCovarianceArrayStamped.h>
#include <mrs_msgs/DirectionWithCovarianceArrayStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <mrs_msgs/NavSatFixArrayStamped.h>
#include <mrs_msgs/NavSatFixIdentified.h>
#include <sensor_msgs/NavSatFix.h>

#include <uwb_range/BeaconStamped.h>
#include <uwb_range/Beacon.h>
#include <std_msgs/String.h>

#include <mrs_lib/gps_conversions.h>
#include <mrs_lib/transformer.h>
#include <mrs_lib/param_loader.h>

#include "tracker.h"
#include "helperfun.h"

#define DEFAULT_OUTPUT_FRAMERATE 20.0
#define MIN_MEASUREMENTS_TO_VALIDATION 5
#define POS_THRESH 2.0
#define MAH_THRESH 2.0
#define YAW_THRESH 1.5
#define MATCH_LEVEL_THRESHOLD_ASSOCIATE 0.3
#define MATCH_LEVEL_THRESHOLD_REMOVE 0.5

std::unordered_map<uint64_t, std::shared_ptr<Tracker>> tracker_map;
std::shared_ptr<mrs_lib::Transformer> transformer;

ros::Publisher pose_debug;
ros::Publisher publish_pose;
ros::Publisher uav_status;
ros::Publisher publish_velocity;
std::string kalman_frame;
std::string gps_frame;

int kalman_pose_model;
int kalman_rotation_model;
double spectral_density_pose;
double spectral_density_rotation;
double time_to_live;

bool use_uvdar, use_uwb, use_beacon;

// contains code from https://gist.github.com/PeteBlackerThe3rd/f73e9d569e29f23e8bd828d7886636a0
Eigen::Quaterniond getAverageOrientation(Eigen::MatrixXd quats, Eigen::VectorXd weights)
{
    if (weights.size() < 1)
    {
        return Eigen::Quaterniond::Identity();
        ROS_ERROR_STREAM("[UVDARPoseCalculator]: No measurements provided. Returning!");
    }

    // first build a 4x4 matrix which is the elementwise sum of the product of each quaternion with itself
    Eigen::Matrix4d A = Eigen::Matrix4d::Zero();

    for (int i = 0; i < weights.size(); ++i)
    {
        A += weights(i) * quats.col(i) * quats.col(i).transpose();
    }

    // Compute the SVD of this 4x4 matrix
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);

    Eigen::VectorXd singularValues = svd.singularValues();
    Eigen::MatrixXd U = svd.matrixU();

    // find the eigen vector corresponding to the largest eigen value
    int largestEigenValueIndex = 0;
    double largestEigenValue = singularValues(0);

    for (int i = 1; i < singularValues.rows(); ++i)
    {
        if (singularValues(i) > largestEigenValue)
        {
            largestEigenValue = singularValues(i);
            largestEigenValueIndex = i;
        }
    }

    /* if (largestEigenValueIndex == -1){ */
    /*   ROS_ERROR("[UVDARPoseCalculator]: Failed to obtain orientation mean. Returning!"); */
    /*   return std::pair<std::pair<e::Vector3d, e::Quaterniond>, e::MatrixXd>(); */
    /* } */

    Eigen::Quaterniond mean_rot;
    mean_rot.x() = U(0, largestEigenValueIndex);
    mean_rot.y() = U(1, largestEigenValueIndex);
    mean_rot.z() = U(2, largestEigenValueIndex);
    mean_rot.w() = U(3, largestEigenValueIndex);

    return mean_rot;
}

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

        if (not tracker->get_is_valid() and tracker->get_total_count() < MIN_MEASUREMENTS_TO_VALIDATION)
            continue;

        std::pair<kalman::x_t, kalman::P_t> result = tracker->predict(msg.header.stamp);

        geometry_msgs::PoseWithCovariance pose = tracker->get_PoseWithCovariance(result.first, result.second);

        pose_identified.id = id;
        pose_identified.pose = pose.pose;
        pose_identified.covariance = pose.covariance;

        msg.poses.push_back(pose_identified);

        // geometry_msgs::TwistStamped velocity;
        // velocity.twist = tracker->get_TwistWithCovariance(result.first, result.second).twist;

        // velocity.header = msg.header;

        // publish_velocity.publish(velocity);
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
        msg_status.data = std::string("-id error_msg -R [OT] got old UVDAR").c_str();
        uav_status.publish(msg_status);
        return;
    }

    std::optional<geometry_msgs::TransformStamped> transformation;

    if (msg.header.frame_id != kalman_frame)
    {
        transformation = transformer->getTransform(msg.header.frame_id, kalman_frame, stamp);

        if (not transformation)
        {
            ROS_WARN("[OBJECT TRACKER] Not found any transformation");
            return;
        }
    }

    // Get vector of measurements[i].id
    std::vector<uint64_t> ids;
    // Define a lambda function to extract 'id' from each measurement
    auto getId = [](const auto &measurement)
    { return measurement.id; };
    // Vectorize the loop using std::transform
    std::transform(msg.poses.begin(), msg.poses.end(), std::back_inserter(ids), getId);

    // Get unique ids
    std::sort(ids.begin(), ids.end());
    ids.erase(std::unique(ids.begin(), ids.end()), ids.end());

    // for (auto const &id : ids)
    // {
    //     ROS_INFO_THROTTLE(0.5, "[OBJECT TRACKER] POSE measurement with ID 0x%lX", id);

    //     std::vector<mrs_msgs::PoseWithCovarianceIdentified> measurements;
    //     std::copy_if(msg.poses.begin(), msg.poses.end(), std::back_inserter(measurements), [id](const auto &measurement)
    //                  { return measurement.id == id; });

    //     bool tracker_exists = tracker_map.count(id);

    //     if (not tracker_exists)
    //     {
    //         ROS_INFO("[OBJECT TRACKER] Creating new tracker for object ID: 0x%lX", id);
    //         tracker_map[id] = std::make_shared<Tracker>(kalman_pose_model,
    //                                                     kalman_rotation_model,
    //                                                     spectral_density_pose,
    //                                                     spectral_density_rotation,
    //                                                     transformer);

    //         std::stringstream ss;
    //         std_msgs::String msg_status;

    //         ss << "-id create_msg -g [OT] Creating tracker "
    //            << "0x" << std::setfill('0') << std::setw(2) << std::right << std::hex << id;
    //         auto x = ss.str();
    //         msg_status.data = x.c_str();
    //         uav_status.publish(msg_status);
    //     }

    //     auto tracker = tracker_map[id];

    //     std::pair<kalman::x_t, kalman::P_t> result = tracker->predict(stamp);

    //     auto reduced_result = statecovReduce(result.first, result.second, 1);

    //     Eigen::Vector3d predicted_pose = reduced_result.first.head(3);
    //     Eigen::Matrix3d predicted_cov = reduced_result.second.topLeftCorner<3, 3>();

    //     Eigen::MatrixXd poses = Eigen::MatrixXd::Zero(3, measurements.size());
    //     Eigen::MatrixXd orientations = Eigen::MatrixXd::Zero(4, measurements.size());
    //     Eigen::VectorXd weights = Eigen::VectorXd::Zero(measurements.size());

    //     for (std::size_t i = 0; i < measurements.size(); ++i)
    //     {
    //         // convert original msg to stamped pose
    //         auto const &measurement = measurements[i];

    //         auto pose_stamped = poseIdentifiedToPoseStamped(measurement);
    //         pose_stamped.header = msg.header;

    //         if (msg.header.frame_id != kalman_frame)
    //         {
    //             // transform coordinates from camera to target frame
    //             std::optional<geometry_msgs::PoseWithCovarianceStamped> pose_transformed;
    //             pose_transformed = transformer->transform(pose_stamped, transformation.value());
    //             if (not pose_transformed)
    //                 continue;

    //             pose_stamped = pose_transformed.value();
    //         }

    //         auto &orientation = pose_stamped.pose.pose.orientation;
    //         auto &position = pose_stamped.pose.pose.position;

    //         Eigen::Quaterniond quaternion(
    //             orientation.w,
    //             orientation.x,
    //             orientation.y,
    //             orientation.z);

    //         // Write position to poses matrix
    //         poses.col(i) = Eigen::Vector3d(position.x, position.y, position.z);
    //         // Write orientation to poses matrix
    //         orientations.col(i) = Eigen::Vector4d(quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w());

    //         double mh = sqrt((poses.col(i) - predicted_pose).transpose() * predicted_cov.inverse() * (poses.col(i) - predicted_pose));

    //         // Add mh to wight matrix
    //         weights(i) = exp(-mh);
    //     }

    //     weights /= weights.sum();

    //     Eigen::Vector3d pose = poses * weights;
    //     auto quat = getAverageOrientation(orientations, weights);

    //     poses.colwise() -= pose;

    //     auto P = (poses * weights.asDiagonal()) * poses.transpose();

    //     kalman::pose_lkf_t::z_t z;
    //     kalman::pose_lkf_t::R_t R = kalman::pose_lkf_t::R_t::Identity();
    //     z << pose(0), pose(1), pose(2), 0, 0, 0;

    //     z(3) = fixAngle(quatToRoll(quat), 0);
    //     z(4) = fixAngle(quatToPitch(quat), 0);
    //     z(5) = fixAngle(quatToYaw(quat), 0);
    //     R.topLeftCorner<3, 3>() = P;

    //     geometry_msgs::PoseStamped debug;

    //     debug.pose.position.x = z(0);
    //     debug.pose.position.y = z(1);
    //     debug.pose.position.z = z(2);

    //     debug.pose.orientation.w = quat.w();
    //     debug.pose.orientation.x = quat.x();
    //     debug.pose.orientation.y = quat.y();
    //     debug.pose.orientation.z = quat.z();

    //     debug.header.frame_id = kalman_frame;
    //     debug.header.stamp = stamp;

    //     pose_debug.publish(debug);

    //     tracker->addMeasurement(stamp, z, R);
    // }

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
            ROS_INFO("[OBJECT TRACKER] Creating new tracker for object ID: 0x%lX", measurement.id);
            tracker_map[measurement.id] = std::make_shared<Tracker>(kalman_pose_model,
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

        ROS_INFO("[OBJECT TRACKER] Adding measurement: %.1f m | %.1f m | ID 0x%lX", pose_vector(0), pose_vector(1), measurement.id);

        kalman::pose_lkf_t::z_t z(pose_vector);
        kalman::pose_lkf_t::R_t R(covariance);

        tracker->addMeasurement(stamp, z, R);
    }
    return;
}

void beacon_callback(const uwb_range::BeaconStampedConstPtr &msg)
{
    if (use_beacon == false)
        return;

    ros::Time stamp = msg->header.stamp;
    ros::Duration age = ros::Time::now() - stamp;

    if (age.toSec() > time_to_live)
    {
        ROS_WARN("[OBJECT TRACKER] Beacon message is %.3f sec old", age.toSec());
        std_msgs::String msg_status;
        msg_status.data = std::string("-id error_msg -R [OT] got old BEACON message").c_str();
        uav_status.publish(msg_status);
        return;
    }

    std::optional<geometry_msgs::TransformStamped> transformation;

    transformation = transformer->getTransform(msg->header.frame_id, kalman_frame, stamp);

    if (not transformation)
    {
        ROS_WARN("[OBJECT TRACKER] Not found any transformation from GPS to kalman frame");
        return;
    }

    ROS_INFO_THROTTLE(1.0, "[OBJECT TRACKER] Beacon message ID 0x%X", msg->beacon.id);

    // create new Tracker instace if not available yet
    if (not tracker_map.count(msg->beacon.id))
    {
        ROS_INFO("[OBJECT TRACKER] Creating new tracker for object ID: 0x%X", msg->beacon.id);
        tracker_map[msg->beacon.id] = std::make_shared<Tracker>(kalman_pose_model,
                                                                kalman_rotation_model,
                                                                spectral_density_pose,
                                                                spectral_density_rotation,
                                                                transformer);

        std::stringstream ss;
        std_msgs::String msg_status;

        ss << "-id create_msg -g [OT] Creating tracker "
           << "0x" << std::setfill('0') << std::setw(2) << std::right << std::hex << msg->beacon.id;
        auto x = ss.str();
        msg_status.data = x.c_str();
        uav_status.publish(msg_status);
    }

    kalman::beacon_ukf_t::z_t z = kalman::beacon_ukf_t::z_t::Zero();
    kalman::beacon_ukf_t::R_t R = 1000 * kalman::beacon_ukf_t::R_t::Identity();

    if (std::isfinite(msg->beacon.LAT) and std::isfinite(msg->beacon.LON))
    {
        geometry_msgs::PointStamped point;
        point.point.z = msg->beacon.ALT;

        mrs_lib::UTM(msg->beacon.LAT, msg->beacon.LON, &point.point.x, &point.point.y);

        std::optional<geometry_msgs::PointStamped> point_transformed = transformer->transform(point, transformation.value());
        if (point_transformed)
        {
            point = point_transformed.value();

            z(0) = point.point.x;
            z(1) = point.point.y;

            R(0, 0) = 1;
            R(1, 1) = 1;
        }
        else
        {
            ROS_WARN("[OBJECT TRACKER] Could not transform GPS measurement to kalman frame");
        }
    }
    else
    {
        ROS_ERROR_THROTTLE(1.0, "[OBJECT TRACKER] NaN detected in GPS coord!!!");
        return;
    }

    if (std::isfinite(msg->beacon.ALT))
    {
        z(2) = msg->beacon.ALT;
        R(2, 2) = 2;
    }

    if (std::isfinite(msg->beacon.heading))
    {
        geometry_msgs::TransformStamped transform = transformation.value();

        z(3) = msg->beacon.heading + tf::getYaw(transform.transform.rotation);
        z(3) = fmod(z(3) + M_PI, 2 * M_PI) - M_PI;
        R(3, 3) = 0.1;
    }

    geometry_msgs::PoseStamped pose;

    pose.pose.position.x = z(0);
    pose.pose.position.y = z(1);
    pose.pose.position.z = z(2);

    Eigen::Quaterniond quat;
    quat = Eigen::AngleAxisd(z(3), Eigen::Vector3d::UnitZ()) *
           Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
           Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());

    pose.pose.orientation.w = quat.w();
    pose.pose.orientation.x = quat.x();
    pose.pose.orientation.y = quat.y();
    pose.pose.orientation.z = quat.z();

    pose.header.frame_id = kalman_frame;
    pose.header.stamp = stamp;

    pose_debug.publish(pose);

    auto tracker = tracker_map[msg->beacon.id];
    tracker->set_valid();

    tracker->addMeasurement(stamp, z, R);

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
            transformation = transformer->getTransform(msg.header.frame_id, kalman_frame, stamp);
        }
        catch (std::exception &e)
        {
            ROS_WARN_STREAM("[OBJECT TRACKER] Exception: " << e.what());
        }

        if (not transformation)
        {
            if (ros::Time::now() - stamp < ros::Duration(0.1))
            {
                transformation = transformer->getTransform(msg.header.frame_id, kalman_frame, ros::Time(0));
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

        if (isnan(measurement.range.range) or measurement.range.max_range < measurement.range.range or measurement.range.min_range > measurement.range.range)
        {
            ROS_ERROR("[OBJECT TRACKER] Invalid range measurement for tracker 0x%lX", measurement.id);
            continue;
        }

        tracker->set_valid();

        kalman::range_ukf_t::z_t z(1 * measurement.range.range);
        kalman::range_ukf_t::R_t R(1 * measurement.variance);

        tracker->addMeasurement(stamp, z, R, transformation.value());
    }

    return;
}

void direction_callback(const mrs_msgs::DirectionWithCovarianceArrayStamped::ConstPtr &msg)
{
    if (msg->directions.empty())
        return;

    ROS_DEBUG("[OBJECT TRACKER] Getting %ld direction measurements", msg->directions.size());

    ros::Time stamp = msg->header.stamp;
    ros::Duration age = ros::Time::now() - stamp;

    if (age.toSec() > time_to_live)
    {
        ROS_WARN("[OBJECT TRACKER] Direction message is %.3f sec old", age.toSec());
        std_msgs::String msg_status;
        msg_status.data = std::string("-id error_msg -R [OT] got old DIRECTION").c_str();
        uav_status.publish(msg_status);
        return;
    }

    std::optional<geometry_msgs::TransformStamped> transformation;

    if (msg->header.frame_id != kalman_frame)
    {
        try
        {
            transformation = transformer->getTransform(msg->header.frame_id, kalman_frame, stamp);
        }
        catch (std::exception &e)
        {
            ROS_WARN_STREAM("[OBJECT TRACKER] Exception: " << e.what());
        }

        if (not transformation)
        {
            if (ros::Time::now() - stamp < ros::Duration(0.1))
            {
                transformation = transformer->getTransform(msg->header.frame_id, kalman_frame, ros::Time(0));
            }
            if (not transformation)
            {
                ROS_WARN("[OBJECT TRACKER] Not found any transformation, exiting");
                return;
            }
        }
    }

    for (auto const &measurement : msg->directions)
    {
        // if (not tracker_map.count(measurement.id))
        //     continue;

        // create new Tracker instace if not available yet
        if (not tracker_map.count(measurement.id))
        {
            ROS_INFO("[OBJECT TRACKER] Creating new tracker for object ID: 0x%X", measurement.id);
            tracker_map[measurement.id] = std::make_shared<Tracker>(kalman_pose_model,
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

        ROS_INFO_THROTTLE(1.0, "[OBJECT TRACKER] Direction measurement for ID 0x%lX", measurement.id);

        auto tracker = tracker_map[measurement.id];

        tracker->set_valid();

        kalman::direction_ukf_t::z_t z(measurement.direction.x, measurement.direction.y, measurement.direction.z);
        kalman::direction_ukf_t::R_t R;

        R << measurement.covariance[0], measurement.covariance[1], measurement.covariance[2], measurement.covariance[3], measurement.covariance[4], measurement.covariance[5], measurement.covariance[6], measurement.covariance[7], measurement.covariance[8];

        tracker->addMeasurement(stamp, z, R, transformation.value());
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

    mrs_lib::ParamLoader param_loader(nh, "Object tracker");

    param_loader.loadParam("uav_name", uav_name);
    param_loader.loadParam("kalman_frame", kalman_frame, uav_name + std::string("/local_origin"));
    param_loader.loadParam("gps_frame", gps_frame, uav_name + std::string("/world_origin"));
    param_loader.loadParam("output_framerate", output_framerate, double(DEFAULT_OUTPUT_FRAMERATE));

    param_loader.loadParam("kalman_pose_model", kalman_pose_model);
    param_loader.loadParam("kalman_rotation_model", kalman_rotation_model);
    param_loader.loadParam("spectral_density_pose", spectral_density_pose);
    param_loader.loadParam("spectral_density_rotation", spectral_density_rotation);
    param_loader.loadParam("time_to_live", time_to_live);

    param_loader.loadParam("use_uvdar", use_uvdar, true);
    param_loader.loadParam("use_beacon", use_beacon, true);
    param_loader.loadParam("use_uwb", use_uwb, true);

    transformer = std::make_shared<mrs_lib::Transformer>("OBJECT TRACKER", ros::Duration(time_to_live));

    transformer->retryLookupNewest(true);
    transformer->setDefaultPrefix("");

    if (not(use_uvdar or use_beacon))
    {
        ROS_ERROR("[OBJECT TRACKER]: At least one of the sensors must be enabled.");
        return -1;
    }

    ros::Subscriber pose_sub, beacon_sub, range_sub, direction_sub;

    pose_sub = nh.subscribe("uvdar_in", 10, pose_callback);
    beacon_sub = nh.subscribe("beacon_in", 10, beacon_callback);
    range_sub = nh.subscribe("uwb_in", 10, range_callback);
    direction_sub = nh.subscribe<mrs_msgs::DirectionWithCovarianceArrayStamped>("direction_in", 10, direction_callback);

    pose_debug = nh.advertise<geometry_msgs::PoseStamped>("pose_debug", 10);
    publish_pose = nh.advertise<mrs_msgs::PoseWithCovarianceArrayStamped>("filtered_poses", 10);
    uav_status = nh.advertise<std_msgs::String>("uav_status", 1);
    publish_velocity = nh.advertise<geometry_msgs::TwistStamped>("filtered_velocity", 10);

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
