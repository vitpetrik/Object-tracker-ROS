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

Eigen::Vector3d ukf_offset = Eigen::Vector3d::Zero() / 0;
kalman::range_ukf_t::z_t observe_ukf(const kalman::range_ukf_t::x_t &x)
{
    Eigen::VectorXd pose(3);

    pose << x[(int)STATE::X], x[(int)STATE::Y], x[(int)STATE::Z];
    pose -= ukf_offset;

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
    Tracker(TRANSITION_MODEL_TYPE::CONSTANT_POSITION, TRANSITION_MODEL_TYPE::CONSTANT_POSITION, 1, 1);
}

Tracker::Tracker(int position_model, int rotation_model, double spectral_density_pose, double spectral_density_rotation, std::shared_ptr<mrs_lib::Transformer> tf_ptr)
{
    this->transformer = tf_ptr;

    this->position_model_type = position_model;
    this->rotation_model_type = rotation_model;

    this->spectral_density_pose = spectral_density_pose;
    this->spectral_density_rotation = spectral_density_rotation;

    this->range_count = 0;
    this->pose_count = 0;

    this->history_map = history_map_t();

    this->initializeFilters();

    return;
}

Tracker::~Tracker()
{
    return;
}

void Tracker::initializeFilters()
{
    kalman::pose_lkf_t::H_t H_matrix_pose = kalman::pose_lkf_t::H_t::Zero();

    H_matrix_pose(0, (int)STATE::X) = 1;
    H_matrix_pose(1, (int)STATE::Y) = 1;
    H_matrix_pose(2, (int)STATE::Z) = 1;
    H_matrix_pose(3, (int)STATE::ROLL) = 1;
    H_matrix_pose(4, (int)STATE::PITCH) = 1;
    H_matrix_pose(5, (int)STATE::YAW) = 1;

    kalman::pose2d_lkf_t::H_t H_matrix_pose2d = kalman::pose2d_lkf_t::H_t::Zero();

    H_matrix_pose2d(0, (int)STATE::X) = 1;
    H_matrix_pose2d(1, (int)STATE::Y) = 1;

    kalman::pose3d_lkf_t::H_t H_matrix_pose3d = kalman::pose3d_lkf_t::H_t::Zero();

    H_matrix_pose3d(0, (int)STATE::X) = 1;
    H_matrix_pose3d(1, (int)STATE::Y) = 1;
    H_matrix_pose3d(2, (int)STATE::Z) = 1;

    this->pose_lkf = kalman::pose_lkf_t(kalman::pose_lkf_t::A_t::Identity(),
                                        kalman::pose_lkf_t::B_t::Zero(),
                                        H_matrix_pose);
    this->pose2d_lkf = kalman::pose2d_lkf_t(kalman::pose2d_lkf_t::A_t::Identity(),
                                            kalman::pose2d_lkf_t::B_t::Zero(),
                                            H_matrix_pose2d);
    this->pose3d_lkf = kalman::pose3d_lkf_t(kalman::pose3d_lkf_t::A_t::Identity(),
                                            kalman::pose3d_lkf_t::B_t::Zero(),
                                            H_matrix_pose3d);
    this->predict_lkf = kalman::predict_lkf_t(kalman::predict_lkf_t::A_t::Identity(),
                                              kalman::predict_lkf_t::B_t::Zero(),
                                              kalman::predict_lkf_t::H_t::Identity());
    this->range_ukf = kalman::range_ukf_t(transition_ukf, observe_ukf);
    this->range_ukf.setConstants(0.1, 1, 2);

    return;
}

std::pair<kalman::x_t, kalman::P_t> Tracker::predict(ros::Time time)
{
    std::pair<kalman::x_t, kalman::P_t> state = this->get_state();
    kalman::x_t x = state.first;
    kalman::P_t P = state.second;

    double dt = std::fmax((time - this->get_last_correction()).toSec(), 0.0);

    if (dt == 0)
        return std::make_pair(x, P);

    this->predict_lkf.A = transitionMatrix(dt, this->position_model_type, this->rotation_model_type);
    kalman::predict_lkf_t::statecov_t statecov = {x, P, time};

    statecov = this->predict_lkf.predict(statecov, kalman::predict_lkf_t::u_t::Zero(), processNoiseMatrix(dt, this->position_model_type, this->rotation_model_type, this->spectral_density_pose, this->spectral_density_rotation), dt);
    x = statecov.x;
    P = statecov.P;

    return std::make_pair(x, P);
}

void Tracker::runCorrectionFrom(history_map_t::iterator apriori)
{
    if (std::next(apriori) == this->history_map.end())
        return;

    auto x = apriori->second.x;
    auto P = apriori->second.P;

    history_map_t::iterator posteriori = std::next(apriori);
    history_t &history = posteriori->second;
    ros::Duration dt = posteriori->first - apriori->first;

    if (0 < dt.toSec())
    {
        kalman::predict_lkf_t::statecov_t statecov = {x, P, apriori->first};
        this->predict_lkf.A = transitionMatrix(dt.toSec(), this->position_model_type, this->rotation_model_type);
        statecov = this->predict_lkf.predict(statecov,
                                             kalman::predict_lkf_t::u_t::Zero(),
                                             processNoiseMatrix(dt.toSec(), this->position_model_type, this->rotation_model_type, this->spectral_density_pose, this->spectral_density_rotation),
                                             dt.toSec());
        x = statecov.x;
        P = statecov.P;
    }
    else if (dt.toSec() < 0)
    {
        ROS_WARN("Time difference between measurements is Negative!");
    }
    else
    {
        ROS_WARN("Time difference between measurements is Zero!");
    }

    if (history.measurement.index() == 0)
    {
        kalman::pose_lkf_t::z_t z = std::get<0>(history.measurement).z;
        kalman::pose_lkf_t::R_t R = std::get<0>(history.measurement).R;

        x[(int)STATE::ROLL] = fixAngle(x[(int)STATE::ROLL], z[3]);
        x[(int)STATE::PITCH] = fixAngle(x[(int)STATE::PITCH], z[4]);
        x[(int)STATE::YAW] = fixAngle(x[(int)STATE::YAW], z[5]);

        kalman::pose_lkf_t::statecov_t statecov = {x, P, apriori->first};

        try
        {
            statecov = this->pose_lkf.correct(statecov, z, R);
        }
        catch ([[maybe_unused]] std::exception &e)
        {
            ROS_WARN("Could retrieve matrix inversion");
        }

        x = statecov.x;
        P = statecov.P;

        x[(int)STATE::ROLL] = fixAngle(x[(int)STATE::ROLL], 0);
        x[(int)STATE::PITCH] = fixAngle(x[(int)STATE::PITCH], 0);
        x[(int)STATE::YAW] = fixAngle(x[(int)STATE::YAW], 0);

        history.x = x;
        history.P = P;
    }
    else if (history.measurement.index() == 1)
    {
        kalman::pose2d_lkf_t::z_t z = std::get<1>(history.measurement).z;
        kalman::pose2d_lkf_t::R_t R = std::get<1>(history.measurement).R;

        kalman::pose2d_lkf_t::statecov_t statecov = {x, P, apriori->first};

        try
        {
            statecov = this->pose2d_lkf.correct(statecov, z, R);
        }
        catch ([[maybe_unused]] std::exception &e)
        {
            ROS_WARN("Could retrieve matrix inversion");
        }

        x = statecov.x;
        P = statecov.P;

        history.x = x;
        history.P = P;
    }
    else if (history.measurement.index() == 2)
    {
        kalman::pose3d_lkf_t::z_t z = std::get<2>(history.measurement).z;
        kalman::pose3d_lkf_t::R_t R = std::get<2>(history.measurement).R;

        kalman::pose3d_lkf_t::statecov_t statecov = {x, P, apriori->first};

        try
        {
            statecov = this->pose3d_lkf.correct(statecov, z, R);
        }
        catch ([[maybe_unused]] std::exception &e)
        {
            ROS_WARN("Could retrieve matrix inversion");
        }

        x = statecov.x;
        P = statecov.P;

        history.x = x;
        history.P = P;
    }
    else if (history.measurement.index() == 3)
    {
        kalman::range_ukf_t::z_t z = std::get<3>(history.measurement).z;
        kalman::range_ukf_t::R_t R = std::get<3>(history.measurement).R;
        geometry_msgs::TransformStamped transformation = std::get<3>(history.measurement).transformation;

        Eigen::Vector3d translate = Eigen::Vector3d(transformation.transform.translation.x,
                                                    transformation.transform.translation.y,
                                                    transformation.transform.translation.z);
        ukf_offset = translate;

        kalman::range_ukf_t::statecov_t statecov = {x, P, apriori->first};

        try
        {
            statecov = this->range_ukf.correct(statecov, z, R);
        }
        catch ([[maybe_unused]] std::exception &e)
        {
            ROS_WARN("Could retrieve matrix inversion");
        }

        ukf_offset = Eigen::Vector3d::Zero() / 0;

        x = statecov.x;
        P = statecov.P;

        history.x = x;
        history.P = P;
    }
    else
    {
        ROS_WARN("Unknown measurement type");
    }

    this->runCorrectionFrom(posteriori);

    return;
}

std::optional<Tracker::history_map_t::iterator> Tracker::addMeasurement(ros::Time time, measurement_t measurement, kalman::x_t x, kalman::P_t P)
{
    history_t history = {measurement, x, P};

    // Handle empty map
    if (this->history_map.empty())
    {
        if (x.array().isNaN().any() or P.array().isNaN().any())
            return std::nullopt;

        return std::optional<history_map_t::iterator>(this->history_map.insert(std::make_pair(time, history)));
    }

    history_map_t::iterator bound = this->history_map.lower_bound(time);
    history_map_t::iterator apriori = this->history_map.begin();

    // dont add new measurements to the beginning of the map
    if (bound == this->history_map.begin())
        return std::nullopt;

    apriori = std::prev(bound);
    auto it = this->history_map.insert(std::make_pair(time, history));

    this->runCorrectionFrom(apriori);

    return std::optional<history_map_t::iterator>(it);
}

std::pair<kalman::x_t, kalman::P_t> Tracker::addMeasurement(ros::Time time, kalman::pose_lkf_t::z_t z, kalman::pose_lkf_t::R_t R)
{
    measurement_t measurement = {pose_measurement_t{z, R}};

    kalman::x_t x = kalman::x_t::Zero();
    kalman::P_t P = 1000 * kalman::P_t::Identity();

    x[(int)STATE::X] = z[0];
    x[(int)STATE::Y] = z[1];
    x[(int)STATE::Z] = z[2];
    x[(int)STATE::ROLL] = z[3];
    x[(int)STATE::PITCH] = z[4];
    x[(int)STATE::YAW] = z[5];

    P((int)STATE::X, (int)STATE::X) = R(0, 0);
    P((int)STATE::X, (int)STATE::Y) = R(0, 1);
    P((int)STATE::X, (int)STATE::Z) = R(0, 2);
    P((int)STATE::Y, (int)STATE::Y) = R(1, 1);
    P((int)STATE::Y, (int)STATE::Z) = R(1, 2);
    P((int)STATE::Z, (int)STATE::Z) = R(2, 2);

    P((int)STATE::ROLL, (int)STATE::ROLL) = R(3, 3);
    P((int)STATE::ROLL, (int)STATE::PITCH) = R(3, 4);
    P((int)STATE::ROLL, (int)STATE::YAW) = R(3, 5);
    P((int)STATE::PITCH, (int)STATE::PITCH) = R(4, 4);
    P((int)STATE::PITCH, (int)STATE::YAW) = R(4, 5);
    P((int)STATE::YAW, (int)STATE::YAW) = R(5, 5);

    P.triangularView<Eigen::Lower>() = P.transpose();

    auto it = this->addMeasurement(time, measurement, x, P);

    if (it)
        this->pose_count++;

    return this->get_state();
}

std::pair<kalman::x_t, kalman::P_t> Tracker::addMeasurement(ros::Time time, kalman::pose2d_lkf_t::z_t z, kalman::pose2d_lkf_t::R_t R)
{
    measurement_t measurement = {pose2d_measurement_t{z, R}};

    kalman::x_t x = kalman::x_t::Zero();
    kalman::P_t P = 1000 * kalman::P_t::Identity();

    x[(int)STATE::X] = z[0];
    x[(int)STATE::Y] = z[1];

    P((int)STATE::X, (int)STATE::X) = R(0, 0);
    P((int)STATE::X, (int)STATE::Y) = R(0, 1);
    P((int)STATE::Y, (int)STATE::Y) = R(1, 1);

    P.triangularView<Eigen::Lower>() = P.transpose();

    auto it = this->addMeasurement(time, measurement, x, P);

    if (it)
        this->pose_count++;

    return this->get_state();
}

std::pair<kalman::x_t, kalman::P_t> Tracker::addMeasurement(ros::Time time, kalman::pose3d_lkf_t::z_t z, kalman::pose3d_lkf_t::R_t R)
{
    measurement_t measurement = {pose3d_measurement_t{z, R}};

    kalman::x_t x = kalman::x_t::Zero();
    kalman::P_t P = 1000 * kalman::P_t::Identity();

    x[(int)STATE::X] = z[0];
    x[(int)STATE::Y] = z[1];
    x[(int)STATE::Z] = z[2];

    P((int)STATE::X, (int)STATE::X) = R(0, 0);
    P((int)STATE::X, (int)STATE::Y) = R(0, 1);
    P((int)STATE::X, (int)STATE::Z) = R(0, 2);
    P((int)STATE::Y, (int)STATE::Y) = R(1, 1);
    P((int)STATE::Y, (int)STATE::Z) = R(1, 2);
    P((int)STATE::Z, (int)STATE::Z) = R(2, 2);

    P.triangularView<Eigen::Lower>() = P.transpose();

    auto it = this->addMeasurement(time, measurement, x, P);

    if (it)
        this->pose_count++;

    return this->get_state();
}

std::pair<kalman::x_t, kalman::P_t> Tracker::addMeasurement(ros::Time time, kalman::range_ukf_t::z_t z, kalman::range_ukf_t::R_t R, geometry_msgs::TransformStamped transformation)
{
    measurement_t measurement = {range_measurement_t{z, R, transformation}};
    auto it = this->addMeasurement(time, measurement);

    return this->get_state();
}

geometry_msgs::PoseWithCovariance Tracker::get_PoseWithCovariance(kalman::x_t x, kalman::P_t P_full)
{
    geometry_msgs::PoseWithCovariance pose;

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

geometry_msgs::TwistWithCovariance Tracker::get_TwistWithCovariance(kalman::x_t x, kalman::P_t P_full)
{
    geometry_msgs::TwistWithCovariance twist;

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

std::pair<kalman::x_t, kalman::P_t> Tracker::transform(geometry_msgs::TransformStamped transformation, kalman::x_t x, kalman::P_t P)
{
    geometry_msgs::PoseWithCovarianceStamped pose_stamped;
    geometry_msgs::TwistWithCovarianceStamped twist_stamped;

    pose_stamped.pose = this->get_PoseWithCovariance(x, P);
    // twist_stamped.twist = this->get_TwistWithCovariance(x, P);

    auto pose_transformed = this->transformer->transform(pose_stamped, transformation);

    if (not pose_transformed)
        return std::make_pair(x, P);

    auto pose_vector = poseToVector(pose_transformed.value().pose);
    auto covariance = rosCovarianceToEigen(pose_transformed.value().pose.covariance);

    x[(int)STATE::X] = pose_vector[0];
    x[(int)STATE::Y] = pose_vector[1];
    x[(int)STATE::Z] = pose_vector[2];
    x[(int)STATE::ROLL] = pose_vector[3];
    x[(int)STATE::PITCH] = pose_vector[4];
    x[(int)STATE::YAW] = pose_vector[5];

    P((int)STATE::X, (int)STATE::X) = covariance(0, 0);
    P((int)STATE::X, (int)STATE::Y) = covariance(0, 1);
    P((int)STATE::X, (int)STATE::Z) = covariance(0, 2);
    P((int)STATE::Y, (int)STATE::Y) = covariance(1, 1);
    P((int)STATE::Y, (int)STATE::Z) = covariance(1, 2);
    P((int)STATE::Z, (int)STATE::Z) = covariance(2, 2);
    P((int)STATE::ROLL, (int)STATE::ROLL) = covariance(3, 3);
    P((int)STATE::ROLL, (int)STATE::PITCH) = covariance(3, 4);
    P((int)STATE::ROLL, (int)STATE::YAW) = covariance(3, 5);
    P((int)STATE::PITCH, (int)STATE::PITCH) = covariance(4, 4);
    P((int)STATE::PITCH, (int)STATE::YAW) = covariance(4, 5);
    P((int)STATE::YAW, (int)STATE::YAW) = covariance(5, 5);

    P.triangularView<Eigen::Lower>() = P.transpose();

    return std::make_pair(x, P);
}

int Tracker::delete_old(ros::Time deadline)
{
    int count = 0;

    for (auto it = this->history_map.cbegin(); it != this->history_map.cend() /* not hoisted */; /* no increment */)
    {
        if (deadline < it->first)
            break;

        if (it->second.measurement.index() == 0)
            this->pose_count--;
        else if (it->second.measurement.index() == 1)
            this->range_count--;
        this->history_map.erase(it++); // or "it = m.erase(it)" since C++11
        ++count;
    }

    return count;
}
