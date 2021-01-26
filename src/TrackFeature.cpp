/**
 * @file TrackFeature.cpp
 * @author Rohan Singh (rohansingh.apjss@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2021-01-26
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "TrackFeature.hpp"

namespace object_detector {

TrackFeature::TrackFeature(ros::NodeHandle& nh) {
    nodeHandle = nh;

    circleSub = nodeHandle.subscribe<object_detector::Circle>("/detectedCircle", 10, &TrackFeature::CircleCallback, this);

    odomPub = nodeHandle.advertise<nav_msgs::Odometry>("/circleOdom", 10);
    trajPub = nodeHandle.advertise<nav_msgs::Path>("/circlePath", 10);

    std::vector<double> q, r, p0; 
    nh.param<std::vector<double>>("/Q", q, std::vector<double>());
    nh.param<std::vector<double>>("/R", r, std::vector<double>());
    nh.param<std::vector<double>>("/P0", p0, std::vector<double>());
    if(!nh.getParam("/Q", q)) {
        q = {1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 10, 0,
            0, 0, 0, 10};
        ROS_WARN("Failed to get param Q");
    }

    if(!nh.getParam("/R", r)) {
        r = {10, 0,
            0, 10};
        ROS_WARN("Failed to get param R");
    }

    if(!nh.getParam("/P0", p0)) {
        p0 = {0.1, 0, 0, 0,
              0, 0.1, 0, 0,
              0, 0, 0.1, 0,
              0, 0, 0, 0.1};
        ROS_WARN("Failed to get param P0");
    }

    Q << q[0], q[1], q[2], q[3],
         q[4], q[5], q[6], q[7],
         q[8], q[9], q[10], q[11],
         q[12], q[13], q[14], q[15];

    R << r[0], r[1],
         r[2], r[3];

    P << p0[0], p0[1], p0[2], p0[3],
         p0[4], p0[5], p0[6], p0[7],
         p0[8], p0[9], p0[10], p0[11],
         p0[12], p0[13], p0[14], p0[15];

    double dt = 0.03;   // Just for initialization

    A << 1, 0, dt, 0,
        0, 1, 0, dt,
        0, 0, 1, 0,
        0, 0, 0, 1;

    C << 1, 0, 0, 0,
        0, 1, 0, 0;

    circleKF = KalmanFilter(0.03, A, C, Q, R, P);
}

void TrackFeature::CircleCallback(const object_detector::Circle::ConstPtr& msg) {
    currCircle = *msg;
    currOdom.header = currCircle.header;

    if (initKF == false) {
        startTime = currCircle.header.stamp;
        currTime = currCircle.header.stamp;
        prevTime = currCircle.header.stamp;

        Eigen::VectorXd x0(4);
        x0 << currCircle.x, currCircle.y, 0, 0;
        circleKF.init(0, x0);
        currOdom.pose.pose.position.x = currCircle.x;
        currOdom.pose.pose.position.y = currCircle.y;
        initKF = true;
    } else {
        currTime = currCircle.header.stamp;
        double dt = (currTime.sec - prevTime.sec) + double(currTime.nsec - prevTime.nsec)*1e-9;
        A << 1, 0, dt, 0,
            0, 1, 0, dt,
            0, 0, 1, 0,
            0, 0, 0, 1;
        Eigen::VectorXd y(2);
        y << currCircle.x, currCircle.y;
        circleKF.update(y, dt, A);
        
        Eigen::VectorXd xNew(4);
        xNew = circleKF.state();

        currOdom.pose.pose.position.x = xNew[0];
        currOdom.pose.pose.position.y = xNew[1];
        currOdom.twist.twist.linear.x = xNew[2];
        currOdom.twist.twist.linear.y = xNew[3];

        geometry_msgs::PoseStamped p;
        p.header = currOdom.header;
        p.pose = currOdom.pose.pose;
        traj.push_back(p);

        prevTime = currTime;
    }

    odomPub.publish(currOdom);

    nav_msgs::Path p;
    p.header = currOdom.header;
    p.poses = traj;
    trajPub.publish(p);
}

}