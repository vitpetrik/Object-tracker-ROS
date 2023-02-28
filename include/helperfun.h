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

#include <mrs_msgs/PoseWithCovarianceArrayStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h> 

#include <Eigen/Dense>

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
Eigen::VectorXd poseToVector(const geometry_msgs::PoseWithCovarianceStamped &pose);

/**
 * @brief Reformat ROS format of covariance matrix to matrix format
 * 
 * @param input 
 * @return Eigen::MatrixXd 
 */
Eigen::MatrixXd rosCovarianceToEigen(const boost::array<double, 36> input);

/**
 * @brief further sanitation of the measurment. I don't understand that very much, but it is in fitler.cpp
 * 
 * @param covariance 
 */
void covarianceSanity(Eigen::MatrixXd &covariance);

#endif /* _HELPER_FUN_ */
