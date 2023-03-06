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

#define STATES_NUM 18

enum TRANSITION_MODEL_TYPE
{
    CONSTANT_POSITION,
    CONSTANT_VELOCITY,
    CONSTANT_ACCELERATION
};

enum class STATE
{
    X,
    X_dt,
    X_dt2,
    Y,
    Y_dt,
    Y_dt2,
    Z,
    Z_dt,
    Z_dt2,
    ROLL,
    ROLL_dt,
    ROLL_dt2,
    PITCH,
    PITCH_dt,
    PITCH_dt2,
    YAW,
    YAW_dt,
    YAW_dt2
};

namespace kalman
{
    using range_ukf_t = mrs_lib::UKF<STATES_NUM, 0, 1>;
    using pose_lkf_t = mrs_lib::LKF<STATES_NUM, 0, 6>;

    using A_t = pose_lkf_t::A_t;
    using P_t = pose_lkf_t::P_t;
    using x_t = pose_lkf_t::x_t;
}

/**
 * @brief Implementation of object tracker using Kalman filters from mrs_lib
 *
 */
class Tracker
{
public:
    Tracker();

    Tracker(kalman::pose_lkf_t::z_t z, kalman::pose_lkf_t::R_t R, int position_model, int rotation_model);

    ~Tracker();

    std::pair<kalman::x_t, kalman::P_t> predict(ros::Time, bool apply_update = true);

    std::pair<kalman::x_t, kalman::P_t> correctPose(ros::Time, kalman::pose_lkf_t::z_t, kalman::pose_lkf_t::R_t, bool apply_update = true);

    std::pair<kalman::x_t, kalman::P_t> correctRange(ros::Time, kalman::range_ukf_t::z_t, kalman::range_ukf_t::R_t, bool apply_update = true);

    const auto get_last_correction() const { return last_correction; }
    const auto get_update_count() const { return update_count; }

    const std::pair<kalman::x_t, kalman::P_t> get_state() const { return std::make_pair(state_vector, covariance); };

private:
    int update_count;
    int position_model_type;
    int rotation_model_type;

    kalman::x_t state_vector;
    kalman::P_t covariance;

    kalman::pose_lkf_t pose_lkf;
    kalman::A_t A_matrix;
    kalman::pose_lkf_t::B_t B_matrix;
    kalman::pose_lkf_t::H_t H_matrix;

    kalman::range_ukf_t range_ukf;

    ros::Time last_prediction;
    ros::Time last_correction;
};

#endif /* SRC_OBJECT_TRACKER_INCLUDE_TRACKER */
