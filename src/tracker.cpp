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
#if __INTELLISENSE__
#undef __ARM_NEON
#undef __ARM_NEON__
#endif

#include <ros/ros.h>

#include <math.h>
#include "tracker.h"
#include "helperfun.h"

#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>

kalman::range_ukf_t::x_t transition_ukf(const kalman::range_ukf_t::x_t &x, const kalman::range_ukf_t::u_t &, [[maybe_unused]] double dt)
{
    return x;
}

kalman::range_ukf_t::z_t observe_ukf(const kalman::range_ukf_t::x_t &x)
{
    Eigen::VectorXd pose(3);

    pose << x[(int)STATE::X], x[(int)STATE::Y], x[(int)STATE::Z];
    kalman::range_ukf_t::z_t z;

    z << pose.norm();
    return z;
}

Eigen::MatrixXd modelMatrix(double dt, int model_type)
{
    Eigen::Matrix2d model_block = Eigen::Matrix2d::Zero();
    Eigen::MatrixXd model = Eigen::MatrixXd::Zero(6, 6);

    switch (model_type)
    {
    case TRANSITION_MODEL_TYPE::CONSTANT_POSITION:
        model_block << 1, 0,
                        0, 0;

        break;
    case TRANSITION_MODEL_TYPE::CONSTANT_VELOCITY:
        model_block << 1, dt,
                        0, 1;
        break;
    // case TRANSITION_MODEL_TYPE::CONSTANT_ACCELERATION:
    //     model_block << 1, dt, 0.5*pow(dt, 2),
    //             0, 1, dt,
    //             0, 0, 1;
    //     break;
    }

    model.block<2, 2>(0, 0) = model_block;
    model.block<2, 2>(2, 2) = model_block;
    model.block<2, 2>(4, 4) = model_block;

    return model;
}

Eigen::MatrixXd Q_continuous_white_noise(int dim, double dt, double spectral_density)
{
    Eigen::Matrix2d Q_block = Eigen::Matrix2d::Zero();
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(6, 6);

    if (dim == TRANSITION_MODEL_TYPE::CONSTANT_POSITION)
    {
        Q_block << dt, 0,
            0, 0;
    }
    else if (dim == TRANSITION_MODEL_TYPE::CONSTANT_VELOCITY)
    {
        Q_block << pow(dt, 3) / 3, pow(dt, 2) / 2,
            pow(dt, 2) / 2, dt;
    }
    // else if (dim == TRANSITION_MODEL_TYPE::CONSTANT_ACCELERATION)
    // {
    //     Q_block << pow(dt, 5) / 20, pow(dt, 4) / 8, pow(dt, 3) / 6,
    //         pow(dt, 4) / 8, pow(dt, 3) / 3, pow(dt, 2) / 2,
    //         pow(dt, 3) / 6, pow(dt, 2) / 2, dt;
    // }

    Q.block<2, 2>(0, 0) = Q_block;
    Q.block<2, 2>(2, 2) = Q_block;
    Q.block<2, 2>(4, 4) = Q_block;

    return spectral_density * Q;
}

kalman::A_t transitionMatrix(double dt, int position_model, int rotation_model)
{
    kalman::A_t A = kalman::A_t::Zero();

    A.topLeftCorner(6, 6) = modelMatrix(dt, position_model);
    A.bottomRightCorner(6, 6) = modelMatrix(dt, rotation_model);

    return A;
}

kalman::predict_lkf_t::Q_t processNoiseMatrix(double dt, int position_model, int rotation_model, double spectral_density_pose, double spectral_density_rotation)
{
    kalman::predict_lkf_t::Q_t Q = kalman::predict_lkf_t::Q_t::Zero();

    Q.topLeftCorner(6, 6) = Q_continuous_white_noise(position_model, dt, spectral_density_pose);
    Q.bottomRightCorner(6, 6) = Q_continuous_white_noise(rotation_model, dt, spectral_density_rotation);

    return Q;
}

Tracker::Tracker()
{
    Tracker(kalman::pose_lkf_t::z_t::Zero(), kalman::pose_lkf_t::R_t::Zero(), TRANSITION_MODEL_TYPE::CONSTANT_POSITION, TRANSITION_MODEL_TYPE::CONSTANT_POSITION, 1, 1);
}

Tracker::Tracker(kalman::pose_lkf_t::z_t z, kalman::pose_lkf_t::R_t R, int position_model, int rotation_model, double spectral_density_pose, double spectral_density_rotation, std::shared_ptr<mrs_lib::Transformer> tf_ptr)
{
    this->update_count = 0;
    this->transformer = tf_ptr;

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
    this->covariance((int)STATE::Y, (int)STATE::Y) = R(1, 1);
    this->covariance((int)STATE::Y, (int)STATE::Z) = R(1, 2);
    this->covariance((int)STATE::Z, (int)STATE::Z) = R(2, 2);

    this->covariance((int)STATE::ROLL, (int)STATE::ROLL) = R(3, 3);
    this->covariance((int)STATE::ROLL, (int)STATE::PITCH) = R(3, 4);
    this->covariance((int)STATE::ROLL, (int)STATE::YAW) = R(3, 5);
    this->covariance((int)STATE::PITCH, (int)STATE::PITCH) = R(4, 4);
    this->covariance((int)STATE::PITCH, (int)STATE::YAW) = R(4, 5);
    this->covariance((int)STATE::YAW, (int)STATE::YAW) = R(5, 5);

    this->covariance.triangularView<Eigen::Lower>() = this->covariance.transpose();
    kalman::pose_lkf_t::H_t H_matrix = kalman::pose_lkf_t::H_t::Zero();

    H_matrix(0, (int)STATE::X) = 1;
    H_matrix(1, (int)STATE::Y) = 1;
    H_matrix(2, (int)STATE::Z) = 1;
    H_matrix(3, (int)STATE::ROLL) = 1;
    H_matrix(4, (int)STATE::PITCH) = 1;
    H_matrix(5, (int)STATE::YAW) = 1;

    this->pose_lkf = kalman::pose_lkf_t(kalman::pose_lkf_t::A_t::Identity(),
                                        kalman::pose_lkf_t::B_t::Zero(),
                                        H_matrix);
    this->predict_lkf = kalman::predict_lkf_t(kalman::predict_lkf_t::A_t::Identity(),
                                              kalman::predict_lkf_t::B_t::Zero(),
                                              kalman::predict_lkf_t::H_t::Identity());
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

    this->predict_lkf.A = transitionMatrix(dt, this->position_model_type, this->rotation_model_type);
    kalman::predict_lkf_t::statecov_t statecov = {x, P, time};

    statecov = this->predict_lkf.predict(statecov, kalman::predict_lkf_t::u_t::Zero(), processNoiseMatrix(dt, this->position_model_type, this->rotation_model_type, this->spectral_density_pose, this->spectral_density_rotation), dt);
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

std::pair<kalman::pose_lkf_t::x_t, kalman::pose_lkf_t::P_t> Tracker::correctPose(ros::Time time, kalman::pose_lkf_t::z_t z, kalman::pose_lkf_t::R_t R, bool apply_update)
{
    kalman::pose_lkf_t::x_t x = this->state_vector;
    kalman::pose_lkf_t::P_t P = this->covariance;

    x[(int)STATE::ROLL] = fixAngle(x[(int)STATE::ROLL], z[3]);
    x[(int)STATE::PITCH] = fixAngle(x[(int)STATE::PITCH], z[4]);
    x[(int)STATE::YAW] = fixAngle(x[(int)STATE::YAW], z[5]);

    kalman::pose_lkf_t::statecov_t statecov = {x, P, time};

    try
    {
        statecov = this->pose_lkf.correct(statecov, z, R);
    }
    catch ([[maybe_unused]] std::exception &e)
    {
        ROS_WARN("Could retrieve matrix inversion");
        return std::make_pair(x, P);
    }

    x = statecov.x;
    P = statecov.P;

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

std::pair<kalman::range_ukf_t::x_t, kalman::range_ukf_t::P_t> Tracker::correctRange(ros::Time time, kalman::range_ukf_t::z_t z, kalman::range_ukf_t::R_t R, bool apply_update)
{
    kalman::range_ukf_t::x_t x = this->state_vector;
    kalman::range_ukf_t::P_t P = this->covariance;

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

        // this->last_correction = time;
        // this->update_count++;
    }

    return std::make_pair(x, P);
}

geometry_msgs::PoseWithCovariance Tracker::get_PoseWithCovariance()
{
    geometry_msgs::PoseWithCovariance pose;

    auto x = this->state_vector;
    auto P_full = this->covariance;

    Eigen::Matrix<double, 6, 6> P = covGetPose(P_full);

    pose.pose.position.x = x[(int)STATE::X];
    pose.pose.position.y = x[(int)STATE::Y];
    pose.pose.position.z = x[(int)STATE::Z];

    Eigen::Quaterniond quaternion = Eigen::AngleAxisd(x[(int)STATE::ROLL], Eigen::Vector3d::UnitX()) *
                    Eigen::AngleAxisd(x[(int)STATE::PITCH], Eigen::Vector3d::UnitY()) *
                    Eigen::AngleAxisd(x[(int)STATE::YAW], Eigen::Vector3d::UnitZ());

    pose.pose.orientation.x = quaternion.x();
    pose.pose.orientation.y = quaternion.y();
    pose.pose.orientation.z = quaternion.z();
    pose.pose.orientation.w = quaternion.w();

    pose.covariance = eigenCovarianceToRos(P);

    return pose;
}

geometry_msgs::TwistWithCovariance Tracker::get_TwistWithCovariance()
{
    geometry_msgs::TwistWithCovariance twist;

    auto x = this->state_vector;
    auto P_full = this->covariance;

    Eigen::Matrix<double, 6, 6> P = covGetVelocity(P_full);

    twist.twist.linear.x = x[(int)STATE::X_dt];
    twist.twist.linear.y = x[(int)STATE::Y_dt];
    twist.twist.linear.z = x[(int)STATE::Z_dt];

    twist.twist.angular.x = x[(int)STATE::ROLL_dt];
    twist.twist.angular.y = x[(int)STATE::PITCH_dt];
    twist.twist.angular.z = x[(int)STATE::YAW_dt];

    twist.covariance = eigenCovarianceToRos(P);

    return twist;
}

bool Tracker::transform(geometry_msgs::TransformStamped transformation)
{
    geometry_msgs::PoseWithCovarianceStamped pose_stamped;
    geometry_msgs::TwistWithCovarianceStamped twist_stamped;

    pose_stamped.pose = this->get_PoseWithCovariance();
    twist_stamped.twist = this->get_TwistWithCovariance();

    auto pose_transformed = this->transformer->transform(pose_stamped, transformation);
    // auto twist_transformed = this->transformer->transform(twist_stamped, transformation);

    if (not pose_transformed)
        return false;

    auto pose_vector = poseToVector(pose_transformed.value().pose);
    auto covariance = rosCovarianceToEigen(pose_transformed.value().pose.covariance);

    this->state_vector[(int)STATE::X] = pose_vector[0];
    this->state_vector[(int)STATE::Y] = pose_vector[1];
    this->state_vector[(int)STATE::Z] = pose_vector[2];
    this->state_vector[(int)STATE::ROLL] = pose_vector[3];
    this->state_vector[(int)STATE::PITCH] = pose_vector[4];
    this->state_vector[(int)STATE::YAW] = pose_vector[5];

    this->covariance((int)STATE::X, (int)STATE::X) = covariance(0, 0);
    this->covariance((int)STATE::X, (int)STATE::Y) = covariance(0, 1);
    this->covariance((int)STATE::X, (int)STATE::Z) = covariance(0, 2);
    this->covariance((int)STATE::Y, (int)STATE::Y) = covariance(1, 1);
    this->covariance((int)STATE::Y, (int)STATE::Z) = covariance(1, 2);
    this->covariance((int)STATE::Z, (int)STATE::Z) = covariance(2, 2);

    this->covariance((int)STATE::ROLL, (int)STATE::ROLL) = covariance(3, 3);
    this->covariance((int)STATE::ROLL, (int)STATE::PITCH) = covariance(3, 4);
    this->covariance((int)STATE::ROLL, (int)STATE::YAW) = covariance(3, 5);
    this->covariance((int)STATE::PITCH, (int)STATE::PITCH) = covariance(4, 4);
    this->covariance((int)STATE::PITCH, (int)STATE::YAW) = covariance(4, 5);
    this->covariance((int)STATE::YAW, (int)STATE::YAW) = covariance(5, 5);

    this->covariance.triangularView<Eigen::Lower>() = this->covariance.transpose();

    return true;
}
