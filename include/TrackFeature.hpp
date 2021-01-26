/**
 * @file TrackFeature.hpp
 * @author Rohan Singh (rohansingh.apjss@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2021-01-26
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "utility.h"

namespace object_detector {

class TrackFeature {
  private:
    ros::NodeHandle nodeHandle;

    ros::Subscriber circleSub;

    ros::Publisher odomPub, trajPub;

    KalmanFilter circleKF;

    Eigen::Matrix<double, 4, 4> A; // System dynamics matrix
    Eigen::Matrix<double, 2, 4> C; // Output matrix
    Eigen::Matrix<double, 4, 4> Q; // Process noise covariance
    Eigen::Matrix<double, 2, 2> R; // Measurement noise covariance
    Eigen::Matrix<double, 4, 4> P; // Estimate error covariance

    bool initKF = false;
    ros::Time currTime;
    ros::Time prevTime;
    ros::Time startTime;

    Circle currCircle;

    nav_msgs::Odometry currOdom;
    std::vector<geometry_msgs::PoseStamped> traj;

  public:
    /**
     * @brief Construct a new Track Feature object
     * 
     * @param nh 
     */
    TrackFeature(ros::NodeHandle& nh);

    /**
     * @brief Calback for detected circle
     * 
     * @param msg 
     */
    void CircleCallback(const object_detector::Circle::ConstPtr& msg);
};

}