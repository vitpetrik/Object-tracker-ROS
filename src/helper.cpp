/**
 * @file helper.cpp
 * @author Vit Petrik (petrivi2@fel.cvut.cz)
 * @brief
 * @version 0.1
 * @date 2023-02-27
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "helperfun.h"

/**
 * @brief Convert PoseWithCovarianceIdentified to PoseWithCovarianceStamped
 *
 * @param pose
 * @return geometry_msgs::PoseWithCovarianceStamped
 */
geometry_msgs::PoseWithCovarianceStamped poseIdentifiedToPoseStamped(const mrs_msgs::PoseWithCovarianceIdentified &pose)
{
    geometry_msgs::PoseWithCovarianceStamped pose_stamped;

    pose_stamped.pose.pose = pose.pose;
    pose_stamped.pose.covariance = pose.covariance;

    return pose_stamped;
}

/**
 * @brief Changes the expression of the origAngle, such that if it is updated in a filtering process with newAngle, circularity issues will be avoided
 *
 * @param origAngle The angle, representing prior state, to be updated
 * @param newAngle The new angle, representing new measurement, that affects origAngle in filtering
 *
 * @return The new expression of origAngle
 */
double fixAngle(double origAngle, double newAngle)
{
    double fixedPre;
    if ((origAngle > (2 * M_PI)) || (origAngle < (-2 * M_PI)))
    {
        fixedPre = fmod(origAngle, 2 * M_PI);
    }
    else
    {
        fixedPre = origAngle;
    }

    if (fixedPre > (M_PI))
        fixedPre = fixedPre - (2.0 * M_PI);
    if (fixedPre < (-M_PI))
        fixedPre = fixedPre + (2.0 * M_PI);

    if (fabs(newAngle - fixedPre) < M_PI)
        return fixedPre;

    if (fixedPre > newAngle)
        return (fixedPre - (2.0 * M_PI));
    else
        return (fixedPre + (2.0 * M_PI));
}

/**
 * @brief Retrieves aviation Roll angle from a quaternion
 *
 * @param q The input quaternion
 *
 * @return The ouput angle
 */
double quatToRoll(Eigen::Quaterniond q)
{
    Eigen::Matrix3d m = q.matrix();
    return atan2(m(2, 1), m(2, 2));
}

/**
 * @brief Retrieves aviation Yaw angle from a quaternion
 *
 * @param q The input quaternion
 *
 * @return The ouput angle
 */
double quatToYaw(Eigen::Quaterniond q)
{
    Eigen::Matrix3d m = q.matrix();
    return atan2(m(1, 0), m(0, 0));
}

/**
 * @brief Retrieves aviation Pitch angle from a quaternion
 *
 * @param q The input quaternion
 *
 * @return The ouput angle
 */
double quatToPitch(Eigen::Quaterniond q)
{
    Eigen::Matrix3d m = q.matrix();
    return atan2(-m(2, 0), sqrt(m(2, 1) * m(2, 1) + m(2, 2) * m(2, 2)));
}

/**
 * @brief Convert pose msg to vector of size 6
 *
 * @param pose
 * @return Eigen::VectorXd
 */
Eigen::VectorXd poseToVector(const geometry_msgs::PoseWithCovarianceStamped &pose)
{
    Eigen::VectorXd pose_vector(6);

    auto &position = pose.pose.pose.position;
    auto &orientation = pose.pose.pose.orientation;

    pose_vector(0) = position.x;
    pose_vector(1) = position.y;
    pose_vector(2) = position.z;

    Eigen::Quaterniond quaternion(
        orientation.w,
        orientation.x,
        orientation.y,
        orientation.z);

    pose_vector(3) = fixAngle(quatToRoll(quaternion), 0);
    pose_vector(4) = fixAngle(quatToPitch(quaternion), 0);
    pose_vector(5) = fixAngle(quatToYaw(quaternion), 0);

    return pose_vector;
}

/**
 * @brief Reformat ROS format of covariance matrix to matrix format
 *
 * @param input
 * @return Eigen::MatrixXd
 */
Eigen::MatrixXd rosCovarianceToEigen(const boost::array<double, 36> input)
{
    Eigen::MatrixXd output(6, 6);

    for (int i = 0; i < 6; i++)
    {
        for (int j = 0; j < 6; j++)
        {
            output(j, i) = input[6 * j + i];
        }
    }

    return output;
}

/**
 * @brief Reformat covarince from matrix form to array
 * 
 * @param input 
 * @return boost::array<double, 36> 
 */
boost::array<double, 36> eigenCovarianceToRos(const Eigen::MatrixXd input)
{
    boost::array<double, 36> output;
    if (((int)(input.rows()) != 6) || ((int)(input.cols()) != 6))
    {
        double n = std::nan("");
        return {n, n, n, n, n, n, n, n, n, n, n, n, n, n, n, n, n, n, n, n, n, n, n, n, n, n, n, n, n, n, n, n, n, n, n, n};
    }

    for (int m = 0; m < 6; m++)
    {
        for (int n = 0; n < 6; n++)
        {
            output[6 * n + m] = input(n, m);
        }
    }

    return output;
}

/**
 * @brief further sanitation of the measurment. I don't understand that very much, but it is in fitler.cpp
 *
 * @param covariance
 */
Eigen::Matrix<double, 6, 6> covarianceSanity(Eigen::Matrix<double, 6, 6> covariance)
{
    Eigen::Matrix<double, 6, 6> P = covariance;

    bool changed = false;
    Eigen::EigenSolver<Eigen::Matrix3d> es(P.bottomRightCorner(3, 3));
    auto eigvals = es.eigenvalues();

    for (int i = 0; i < (int)(eigvals.size()); i++)
    {
        if (eigvals(i).real() >= pow(M_PI, 2))
        {
            eigvals(i) = pow(666.0, 2);
            changed = true;
        }
    }

    if (changed)
    {
        auto eigvecs = es.eigenvectors();
        P.bottomRightCorner(3, 3) = eigvecs.real() * eigvals.real().asDiagonal() * eigvecs.real().transpose();
    }

    return P;
}

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
double gaussJointMaxVal(Eigen::MatrixXd si0, Eigen::MatrixXd si1, Eigen::VectorXd mu0, Eigen::VectorXd mu1)
{
    bool scaled = true;
    int k = mu0.size();
    auto K = si0 * (si0 + si1).inverse();
    auto d0 = K * (mu1 - mu0);
    auto d1 = (K - Eigen::MatrixXd::Identity(K.rows(), K.rows())) * (mu1 - mu0);
    double N;

    if (scaled)
    {
        auto N_v = ((-0.5) * ((d0.transpose() * (si0).inverse() * d0) + (d1.transpose() * (si1).inverse() * d1)));
        N = exp(N_v(0));
    }
    else
    {
        auto N_v = ((-0.5) * ((d0.transpose() * (si0).inverse() * d0) + (d1.transpose() * (si1).inverse() * d1)));
        N = (1.0 / pow((2 * M_PI), k) * sqrt((si0).determinant() * (si1).determinant())) * exp(N_v(0));
    }

    return N;
}
