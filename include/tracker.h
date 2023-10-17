/**
 * @file tracker.h
 * @author Vit Petrik (petrivi2@fel.cvut.cz)
 * @brief Object tracker using kalman filters
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

#ifndef SRC_OBJECT_TRACKER_INCLUDE_TRACKER
#define SRC_OBJECT_TRACKER_INCLUDE_TRACKER

#include <ros/ros.h>
#include <ros/package.h>

#include <map>
#include <variant>
#include <optional>
#include <mutex>

#include <mrs_lib/lkf.h>
#include <mrs_lib/ukf.h>

#include <Eigen/Dense>

#include "helperfun.h"
#include <mrs_lib/transformer.h>

namespace kalman
{
    using pose_lkf_t = mrs_lib::LKF<(int)STATE::STATES_NUM, 0, 6>;
    using beacon_ukf_t = mrs_lib::UKF<(int)STATE::STATES_NUM, 0, 4>;
    using range_ukf_t = mrs_lib::UKF<(int)STATE::STATES_NUM, 0, 1>;
    using predict_lkf_t = mrs_lib::LKF<(int)STATE::STATES_NUM, 0, 6>;

    using A_t = predict_lkf_t::A_t;
    using P_t = predict_lkf_t::P_t;
    using x_t = predict_lkf_t::x_t;
}

/**
 * @brief Implementation of object tracker using Kalman filters from mrs_lib
 *
 */
class Tracker
{
private:
    struct pose_measurement_t
    {
        kalman::pose_lkf_t::z_t z;
        kalman::pose_lkf_t::R_t R;
    };

    struct beacon_measurement_t
    {
        kalman::beacon_ukf_t::z_t z;
        kalman::beacon_ukf_t::R_t R;
    };

    struct range_measurement_t
    {
        kalman::range_ukf_t::z_t z;
        kalman::range_ukf_t::R_t R;

        geometry_msgs::TransformStamped transformation;
    };

    typedef std::variant<pose_measurement_t, beacon_measurement_t, range_measurement_t> measurement_t;

    struct history_t
    {
        measurement_t measurement;

        kalman::x_t x;
        kalman::P_t P;
    };

    using history_map_t = std::multimap<ros::Time, struct history_t>;  

    std::mutex tracker_mutex;

    int position_model_type;
    int rotation_model_type;

    int range_count;
    int pose_count;

    int total_count;
    bool is_valid;

    double spectral_density_pose;
    double spectral_density_rotation;

    kalman::pose_lkf_t pose_lkf;
    kalman::beacon_ukf_t beacon_ukf;
    kalman::range_ukf_t range_ukf;
    kalman::predict_lkf_t predict_lkf;


    std::shared_ptr<mrs_lib::Transformer> transformer;

    history_map_t history_map;

    void runCorrectionFrom(history_map_t::iterator apriori);

    void initializeFilters();

    std::optional<Tracker::history_map_t::iterator> addMeasurement(ros::Time time, measurement_t measurement, kalman::x_t x = kalman::x_t::Zero()/0, kalman::P_t P = kalman::P_t::Zero()/0);


public:
    Tracker();

    Tracker(int position_model, int rotation_model, double spectral_density_pose, double spectral_density_rotation, std::shared_ptr<mrs_lib::Transformer> tf_ptr = nullptr);

    ~Tracker();

    void set_valid(bool valid = true);

    std::pair<kalman::x_t, kalman::P_t> predict(ros::Time);

    std::pair<kalman::x_t, kalman::P_t> addMeasurement(ros::Time, kalman::pose_lkf_t::z_t, kalman::pose_lkf_t::R_t);

    std::pair<kalman::x_t, kalman::P_t> addMeasurement(ros::Time, kalman::beacon_ukf_t::z_t, kalman::beacon_ukf_t::R_t);

    std::pair<kalman::x_t, kalman::P_t> addMeasurement(ros::Time, kalman::range_ukf_t::z_t, kalman::range_ukf_t::R_t, geometry_msgs::TransformStamped transformation);

    const auto get_last_correction() const { return std::prev(history_map.end())->first; }

    const auto get_update_count() const { return history_map.size(); }

    const auto get_pose_count() const { return pose_count; }
    
    const auto get_range_count() const { return range_count; }

    const auto get_total_count() const { return total_count; }

    const auto get_is_valid() const { return is_valid; }

    std::pair<kalman::x_t, kalman::P_t> transform(geometry_msgs::TransformStamped, kalman::x_t, kalman::P_t);

    const std::pair<kalman::x_t, kalman::P_t> get_state() const { return std::make_pair(std::prev(history_map.end())->second.x, std::prev(history_map.end())->second.P); };

    geometry_msgs::PoseWithCovariance get_PoseWithCovariance(kalman::x_t, kalman::P_t);

    geometry_msgs::PoseWithCovariance get_PoseWithCovariance()
    {
        return get_PoseWithCovariance(std::prev(history_map.end())->second.x, std::prev(history_map.end())->second.P);
    }

    geometry_msgs::TwistWithCovariance get_TwistWithCovariance(kalman::x_t, kalman::P_t);

    int delete_old(ros::Time);
};

#endif /* SRC_OBJECT_TRACKER_INCLUDE_TRACKER */
