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

#include <math.h>
#include "tracker.h"
#include "helperfun.h"

kalman::range_ukf_t::x_t transition_ukf(const kalman::range_ukf_t::x_t &x, const kalman::range_ukf_t::u_t &, [[maybe_unused]] double dt)
{
    return x;
}

kalman::range_ukf_t::z_t observe_ukf(const kalman::range_ukf_t::x_t &x)
{
    kalman::range_ukf_t::z_t z = kalman::range_ukf_t::z_t::Zero();

    z << std::sqrt(x[(int)STATE::X] * x[(int)STATE::X] + x[(int)STATE::Y] * x[(int)STATE::Y] + x[(int)STATE::Z] * x[(int)STATE::Z]);

    return z;
}

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

Eigen::MatrixXd Q_continuous_white_noise(int dim, double dt, double spectral_density)
{
    Eigen::Matrix3d Q_block = Eigen::Matrix3d::Zero();
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(9, 9);

    if (dim == TRANSITION_MODEL_TYPE::CONSTANT_POSITION)
    {
        Q_block << dt, 0, 0,
            0, 0, 0,
            0, 0, 0;
    }
    else if (dim == TRANSITION_MODEL_TYPE::CONSTANT_VELOCITY)
    {
        Q_block << pow(dt, 3) / 3, pow(dt, 2) / 2, 0,
            pow(dt, 2) / 2, dt, 0,
            0, 0, 0;
    }
    else if (dim == TRANSITION_MODEL_TYPE::CONSTANT_ACCELERATION)
    {
        Q_block << pow(dt, 5) / 20, pow(dt, 4) / 8, pow(dt, 3) / 6,
            pow(dt, 4) / 8, pow(dt, 3) / 3, pow(dt, 2) / 2,
            pow(dt, 3) / 6, pow(dt, 2) / 2, dt;
    }

    Q.block<3, 3>(0, 0) = Q_block;
    Q.block<3, 3>(3, 3) = Q_block;
    Q.block<3, 3>(6, 6) = Q_block;

    return spectral_density * Q;
}

kalman::A_t transitionMatrix(double dt, int position_model, int rotation_model)
{
    kalman::A_t A = kalman::A_t::Zero();

    A.topLeftCorner(9, 9) = modelMatrix(dt, position_model);
    A.bottomRightCorner(9, 9) = modelMatrix(dt, rotation_model);

    return A;
}

kalman::pose_lkf_t::Q_t processNoiseMatrix(double dt, int position_model, int rotation_model, double spectral_density_pose, double spectral_density_rotation)
{
    kalman::pose_lkf_t::Q_t Q = kalman::pose_lkf_t::Q_t::Zero();

    Q.topLeftCorner(9, 9) = Q_continuous_white_noise(position_model, dt, spectral_density_pose);
    Q.bottomRightCorner(9, 9) = Q_continuous_white_noise(rotation_model, dt, spectral_density_rotation);

    return Q;
}

Tracker::Tracker()
{
    Tracker(kalman::pose_lkf_t::z_t::Zero(), kalman::pose_lkf_t::R_t::Zero(), TRANSITION_MODEL_TYPE::CONSTANT_POSITION, TRANSITION_MODEL_TYPE::CONSTANT_POSITION, 1, 1);
}

Tracker::Tracker(kalman::pose_lkf_t::z_t z, kalman::pose_lkf_t::R_t R, int position_model, int rotation_model, double spectral_density_pose, double spectral_density_rotation)
{
    this->update_count = 0;

    this->position_model_type = position_model;
    this->rotation_model_type = rotation_model;

    this->spectral_density_pose = spectral_density_pose;
    this->spectral_density_rotation = spectral_density_rotation;

    this->state_vector = kalman::x_t::Zero();

    this->state_vector[(int)STATE::X] = z[0];
    this->state_vector[(int)STATE::Y] = z[1];
    this->state_vector[(int)STATE::Z] = z[2];
    this->state_vector[(int)STATE::ROLL] = z[3];
    this->state_vector[(int)STATE::PITCH] = z[4];
    this->state_vector[(int)STATE::YAW] = z[5];

    this->covariance = kalman::P_t::Zero();

    this->covariance((int)STATE::X, (int)STATE::X) = R(0, 0);
    this->covariance((int)STATE::X, (int)STATE::Y) = R(0, 1);
    this->covariance((int)STATE::X, (int)STATE::Z) = R(0, 2);
    this->covariance((int)STATE::Y, (int)STATE::X) = R(1, 0);
    this->covariance((int)STATE::Y, (int)STATE::Y) = R(1, 1);
    this->covariance((int)STATE::Y, (int)STATE::Z) = R(1, 2);
    this->covariance((int)STATE::Z, (int)STATE::X) = R(2, 0);
    this->covariance((int)STATE::Z, (int)STATE::Y) = R(2, 1);
    this->covariance((int)STATE::Z, (int)STATE::Z) = R(2, 2);

    this->covariance((int)STATE::ROLL, (int)STATE::ROLL) = R(3, 3);
    this->covariance((int)STATE::ROLL, (int)STATE::PITCH) = R(3, 4);
    this->covariance((int)STATE::ROLL, (int)STATE::YAW) = R(3, 5);
    this->covariance((int)STATE::PITCH, (int)STATE::ROLL) = R(4, 3);
    this->covariance((int)STATE::PITCH, (int)STATE::PITCH) = R(4, 4);
    this->covariance((int)STATE::PITCH, (int)STATE::YAW) = R(4, 5);
    this->covariance((int)STATE::YAW, (int)STATE::ROLL) = R(5, 3);
    this->covariance((int)STATE::YAW, (int)STATE::PITCH) = R(5, 4);
    this->covariance((int)STATE::YAW, (int)STATE::YAW) = R(5, 5);

    this->A_matrix = kalman::A_t::Zero();
    this->B_matrix = kalman::pose_lkf_t::B_t::Zero();
    this->H_matrix = kalman::pose_lkf_t::H_t::Zero();

    this->H_matrix(0, (int)STATE::X) = 1;
    this->H_matrix(1, (int)STATE::Y) = 1;
    this->H_matrix(2, (int)STATE::Z) = 1;
    this->H_matrix(3, (int)STATE::ROLL) = 1;
    this->H_matrix(4, (int)STATE::PITCH) = 1;
    this->H_matrix(5, (int)STATE::YAW) = 1;

    this->pose_lkf = kalman::pose_lkf_t(this->A_matrix, this->B_matrix, this->H_matrix);
    this->range_ukf = kalman::range_ukf_t(transition_ukf, observe_ukf);

    this->last_prediction = ros::Time::now();
    this->last_correction = ros::Time::now();
    return;
}

Tracker::~Tracker()
{
    return;
}

std::pair<kalman::x_t, kalman::P_t> Tracker::predict(ros::Time time, bool apply_update)
{
    kalman::x_t x = this->state_vector;
    kalman::P_t P = this->covariance;

    double dt = std::fmax(std::fmin((time - this->last_prediction).toSec(), (time - this->last_correction).toSec()), 0.0);

    this->pose_lkf.A = transitionMatrix(dt, this->position_model_type, this->rotation_model_type);
    kalman::pose_lkf_t::statecov_t statecov = {x, P, time};

    statecov = this->pose_lkf.predict(statecov, kalman::pose_lkf_t::u_t::Zero(), processNoiseMatrix(dt, this->position_model_type, this->rotation_model_type, this->spectral_density_pose, this->spectral_density_rotation), dt);
    x = statecov.x;
    P = statecov.P;

    if (apply_update)
    {
        this->state_vector = x;
        this->covariance = P;

        this->last_prediction = time;
    }

    return std::make_pair(x, P);
}

std::pair<kalman::x_t, kalman::P_t> Tracker::correctPose(ros::Time time, kalman::pose_lkf_t::z_t z, kalman::pose_lkf_t::R_t R, bool apply_update)
{
    kalman::x_t x = this->state_vector;
    kalman::P_t P = this->covariance;

    x[(int)STATE::ROLL] = fixAngle(x[(int)STATE::ROLL], z[3]);
    x[(int)STATE::PITCH] = fixAngle(x[(int)STATE::PITCH], z[4]);
    x[(int)STATE::YAW] = fixAngle(x[(int)STATE::YAW], z[5]);

    double match_level_pos = gaussJointMaxVal(
        R.topLeftCorner(3, 3),
        P.topLeftCorner(3, 3),
        z.topRows(3),
        x.topRows(3));

    auto R_temp = R;
    // R_temp.topRightCorner(3, 3).setZero();
    // R_temp.bottomLeftCorner(3, 3).setZero();

    // R_temp.topLeftCorner(3, 3) *= (1.0 / (match_level_pos));

    kalman::pose_lkf_t::statecov_t statecov = {x, P, time};

    try
    {
        statecov = this->pose_lkf.correct(statecov, z, R_temp);
    }
    catch ([[maybe_unused]] std::exception &e)
    {
        ROS_WARN("Could retrieve matrix inversion");
        return std::make_pair(x, P);
    }

    x = statecov.x;
    P = statecov.P;

    // auto eigens_pos = P.topLeftCorner(9, 9).eigenvalues();
    // double min_eig_pos = eigens_pos.real().minCoeff();
    // auto eigens_rot = P.bottomRightCorner(9, 9).eigenvalues();
    // P.topLeftCorner(9, 9) += Eigen::MatrixXd::Identity(9, 9) * (min_eig_pos * (match_level_pos));

    x[(int)STATE::ROLL] = fixAngle(x[(int)STATE::ROLL], 0);
    x[(int)STATE::PITCH] = fixAngle(x[(int)STATE::PITCH], 0);
    x[(int)STATE::YAW] = fixAngle(x[(int)STATE::YAW], 0);

    if (apply_update)
    {
        this->state_vector = x;
        this->covariance = P;

        this->last_correction = time;
        this->update_count++;
    }

    return std::make_pair(x, P);
}

std::pair<kalman::x_t, kalman::P_t> Tracker::correctRange(ros::Time time, kalman::range_ukf_t::z_t z, kalman::range_ukf_t::R_t R, bool apply_update)
{
    kalman::x_t x = this->state_vector;
    kalman::P_t P = this->covariance;

    kalman::range_ukf_t::statecov_t statecov = {x, P, time};

    try
    {
        statecov = this->range_ukf.correct(statecov, z, R);
    }
    catch ([[maybe_unused]] std::exception &e)
    {
        ROS_WARN("Could retrieve matrix inversion");
        return std::make_pair(x, P);
    }

    x = statecov.x;
    P = statecov.P;

    if (apply_update)
    {
        this->state_vector = x;
        this->covariance = P;

        this->last_correction = time;
        this->update_count++;
    }

    return std::make_pair(x, P);
}
