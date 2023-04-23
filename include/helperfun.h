/**
 * @file helperfun.h
 * @author Vit Petrik (petrivi2@fel.cvut.cz)
 * @brief 
 * @version 0.1
 * @date 2023-02-27
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef _HELPERFUN_H_
#define _HELPERFUN_H_


#if __INTELLISENSE__
#undef __ARM_NEON
#undef __ARM_NEON__
#endif

#include <mrs_msgs/PoseWithCovarianceArrayStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h> 
#include <geometry_msgs/PoseWithCovariance.h> 
#include <geometry_msgs/TwistWithCovarianceStamped.h> 
#include <geometry_msgs/TwistWithCovariance.h>

#include <mrs_lib/transformer.h>

#include <Eigen/Dense>
#include <ros/ros.h>


enum TRANSITION_MODEL_TYPE
{
    CONSTANT_POSITION,
    CONSTANT_VELOCITY
};

enum class STATE
{
    X,
    X_dt,
    Y,
    Y_dt,
    Z,
    Z_dt,
    ROLL,
    ROLL_dt,
    PITCH,
    PITCH_dt,
    YAW,
    YAW_dt,
    STATES_NUM
};

/**
 * @brief Convert PoseWithCovarianceIdentified to PoseWithCovarianceStamped
 * 
 * @param pose 
 * @return geometry_msgs::PoseWithCovarianceStamped 
 */
geometry_msgs::PoseWithCovarianceStamped poseIdentifiedToPoseStamped(const mrs_msgs::PoseWithCovarianceIdentified &pose);

/**
 * @brief Changes the expression of the origAngle, such that if it is updated in a filtering process with newAngle, circularity issues will be avoided
 *
 * @param origAngle The angle, representing prior state, to be updated
 * @param newAngle The new angle, representing new measurement, that affects origAngle in filtering
 *
 * @return The new expression of origAngle
 */
double fixAngle(double origAngle, double newAngle);

/**
 * @brief Convert pose msg to vector of size 6
 * 
 * @param pose 
 * @return Eigen::VectorXd 
 */
Eigen::VectorXd poseToVector(const geometry_msgs::PoseWithCovariance &pose);

/**
 * @brief statecov to Pose
 * 
 * @param vector 
 * @return geometry_msgs::PoseWithCovariance 
 */
geometry_msgs::PoseWithCovariance statecovToPose(const Eigen::VectorXd &vector);

/**
 * @brief Reformat ROS format of covariance matrix to matrix format
 * 
 * @param input 
 * @return Eigen::MatrixXd 
 */
Eigen::MatrixXd rosCovarianceToEigen(const boost::array<double, 36> input);

/**
 * @brief Reformat covarince from matrix form to array
 * 
 * @param input 
 * @return boost::array<double, 36> 
 */
boost::array<double, 36> eigenCovarianceToRos(const Eigen::MatrixXd input);

/**
 * @brief Get pose covariance matrix from the big matrix
 * 
 * @param P 
 * @return Eigen::MatrixXd 
 */
Eigen::MatrixXd covGetPose(const Eigen::MatrixXd P);

/**
 * @brief Get velocity covariance matrix from the big matrix
 * 
 * @param P 
 * @return Eigen::MatrixXd 
 */
Eigen::MatrixXd covGetVelocity(const Eigen::MatrixXd P);

/**
 * @brief further sanitation of the measurment. I don't understand that very much, but it is in fitler.cpp
 * 
 * @param covariance 
 */
Eigen::Matrix<double, 6, 6> covarianceSanity(Eigen::Matrix<double, 6, 6> covariance);

/**
 * @brief Returns a value between 0 and 1 representing the level of overlap between two probability distributions in the form of multivariate Gaussians. Multiplying two Gaussians produces another Gaussian, with the value of its peak roughly corresponding to the level of the overlap between the two inputs. The ouptut of this function is the value of such peak, given that the input Gaussians have been scaled s.t. their peaks have the value of 1. Therefore, the output is 1 if the means of both inputs are identical.
 *
 * @param si0 The covariance of the first distribution
 * @param si1 The covariance of the second distribution
 * @param mu0 The mean of the first distribution
 * @param mu1 The mean of the second distribution
 *
 * @return The level of overlap between the two distribution
 */
double gaussJointMaxVal(Eigen::MatrixXd si0, Eigen::MatrixXd si1, Eigen::VectorXd mu0, Eigen::VectorXd mu1);

#endif /* _HELPER_FUN_ */
