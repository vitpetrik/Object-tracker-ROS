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

    auto euler = quaternion.toRotationMatrix().eulerAngles(0, 1, 2);

    pose_vector(3) = fixAngle(euler[0], 0);
    pose_vector(4) = fixAngle(euler[1], 0);
    pose_vector(5) = fixAngle(euler[2], 0);

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
 * @brief further sanitation of the measurment. I don't understand that very much, but it is in fitler.cpp
 * 
 * @param covariance 
 */
void covarianceSanity(Eigen::MatrixXd &covariance)
{
    bool changed = false;
    Eigen::EigenSolver<Eigen::Matrix3d> es(covariance.bottomRightCorner(3, 3));
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
        covariance.bottomRightCorner(3, 3) = eigvecs.real() * eigvals.real().asDiagonal() * eigvecs.real().transpose();
    }

    return;
}
