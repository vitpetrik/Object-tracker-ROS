/**
 * @file tracker.cpp
 * @author Vit Petrik (petrivi2@fel.cvut.cz)
 * @brief
 * @version 0.1
 * @date 2023-02-26
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "tracker.h"

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
    YAW,
    YAW_dt,
    YAW_dt2,
    PITCH,
    PITCH_dt,
    PITCH_dt2
};

Eigen::MatrixXd modelMatrix(double dt, int model_type)
{
    Eigen::MatrixXd model(9, 9);

    switch (model_type)
    {
    case TRANSITION_MODEL_TYPE::CONSTANT_POSITION:
        model << 1, 0, 0, 0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0, 0, 0, 0,
                            0, 0, 0, 1, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0, 1, 0, 0,
                            0, 0, 0, 0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0, 0, 0, 0;
        break;
    case TRANSITION_MODEL_TYPE::CONSTANT_VELOCITY:
        model << 1, dt, 0, 0, 0, 0, 0, 0, 0,
                            0, 1, 0, 0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0, 0, 0, 0,
                            0, 0, 0, 1, dt, 0, 0, 0, 0,
                            0, 0, 0, 0, 1, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0, 1, dt, 0,
                            0, 0, 0, 0, 0, 0, 0, 1, 0,
                            0, 0, 0, 0, 0, 0, 0, 0, 0;
        break;
    case TRANSITION_MODEL_TYPE::CONSTANT_ACCELERATION:
        model << 1, dt, 0.5 * dt * dt, 0, 0, 0, 0, 0, 0,
                            0, 1, dt, 0, 0, 0, 0, 0, 0,
                            0, 0, 1, 0, 0, 0, 0, 0, 0,
                            0, 0, 0, 1, dt, 0.5 * dt * dt, 0, 0, 0,
                            0, 0, 0, 0, 1, dt, 0, 0, 0,
                            0, 0, 0, 0, 0, 1, 0, 0, 0,
                            0, 0, 0, 0, 0, 0, 1, dt, 0.5 * dt * dt,
                            0, 0, 0, 0, 0, 0, 0, 1, dt,
                            0, 0, 0, 0, 0, 0, 0, 0, 1;
        break;
    }

    return model;
}

kalman::A_t transitionMatrix(double dt, int position_model, int rotation_model)
{
    kalman::A_t A = kalman::A_t::Zero();

    A.topLeftCorner(9, 9) = modelMatrix(dt, position_model);
    A.bottomRightCorner(9, 9) = modelMatrix(dt, rotation_model);

    return A;
}

Tracker::Tracker(int position_model, int rotation_model)
{
    this->state_vector = kalman::x_t();
    this->covariance = kalman::P_t();
    this->update_count = 0;

    this->A_matrix = kalman::A_t();
    this->B_matrix = kalman::B_t();
    this->H_matrix = kalman::H_t::Zero();

    this->H_matrix(0, (int) STATE::X) = 1;
    this->H_matrix(1, (int) STATE::Y) = 1;
    this->H_matrix(2, (int) STATE::Z) = 1;
    this->H_matrix(3, (int) STATE::ROLL) = 1;
    this->H_matrix(4, (int) STATE::YAW) = 1;
    this->H_matrix(5, (int) STATE::PITCH) = 1;

    this->pose_lkf = kalman::pose_lkf_t(this->A_matrix, this->B_matrix, this->H_matrix);
    this->range_ukf = kalman::range_ukf_t();

    this->last_prediction = ros::Time::now();
    this->last_correction = ros::Time::now();
    return;
}

Tracker::~Tracker()
{
    return;
}