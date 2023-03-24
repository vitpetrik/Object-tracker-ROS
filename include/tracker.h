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

#ifndef SRC_OBJECT_TRACKER_INCLUDE_TRACKER
#define SRC_OBJECT_TRACKER_INCLUDE_TRACKER

#include <ros/ros.h>
#include <ros/package.h>

#include <mrs_lib/lkf.h>
#include <mrs_lib/ukf.h>

#include <Eigen/Dense>

#include "helperfun.h"

namespace kalman
{
    using range_ukf_t = mrs_lib::UKF<(int)STATE::STATES_NUM, 0, 1>;
    using pose_lkf_t = mrs_lib::LKF<(int)STATE::STATES_NUM, 0, 6>;
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
public:
    Tracker();

    Tracker(kalman::pose_lkf_t::z_t z, kalman::pose_lkf_t::R_t R, int position_model, int rotation_model, double spectral_density_pose, double spectral_density_rotation);

    ~Tracker();

    std::pair<kalman::x_t, kalman::P_t> predict(ros::Time, bool apply_update = true);

    std::pair<kalman::pose_lkf_t::x_t, kalman::pose_lkf_t::P_t> correctPose(ros::Time, kalman::pose_lkf_t::z_t, kalman::pose_lkf_t::R_t, bool apply_update = true);

    std::pair<kalman::range_ukf_t::x_t, kalman::range_ukf_t::P_t> correctRange(ros::Time, kalman::range_ukf_t::z_t, kalman::range_ukf_t::R_t, bool apply_update = true);

    const auto get_last_correction() const { return last_correction; }
    const auto get_update_count() const { return update_count; }

    const std::pair<kalman::x_t, kalman::P_t> get_state() const { return std::make_pair(state_vector, covariance); };

private:
    int update_count;
    int position_model_type;
    int rotation_model_type;

    double spectral_density_pose;
    double spectral_density_rotation;

    kalman::x_t state_vector;
    kalman::P_t covariance;

    kalman::predict_lkf_t predict_lkf;
    kalman::range_ukf_t range_ukf;
    kalman::pose_lkf_t pose_lkf;


    ros::Time last_prediction;
    ros::Time last_correction;
};

#endif /* SRC_OBJECT_TRACKER_INCLUDE_TRACKER */
