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

enum TRANSITION_MODEL_TYPE {
    CONSTANT_POSITION,
    CONSTANT_VELOCITY,
    CONSTANT_ACCELERATION
};

namespace kalman 
{
    using range_ukf_t = mrs_lib::UKF<STATES_NUM, 0, 1>;
    using pose_lkf_t = mrs_lib::LKF<STATES_NUM, 0, 6>;

    using A_t = pose_lkf_t::A_t;
    using B_t = pose_lkf_t::B_t;
    using H_t = pose_lkf_t::H_t;
    using P_t = pose_lkf_t::P_t;
    using x_t = pose_lkf_t::x_t;
}

/**
 * @brief Implementation of object tracker using Kalman filters from mrs_lib
 * 
 */
class Tracker {
    public:
        Tracker(int position_model, int rotation_model);
        ~Tracker();

        int update_count;

    private:
        kalman::x_t state_vector;
        kalman::P_t covariance;

        kalman::A_t A_matrix;
        kalman::B_t B_matrix;
        kalman::H_t H_matrix;

        kalman::range_ukf_t range_ukf;
        kalman::pose_lkf_t pose_lkf;

        ros::Time last_prediction;
        ros::Time last_correction;
};

#endif /* SRC_OBJECT_TRACKER_INCLUDE_TRACKER */
