/**
 * @file ExtractFeatures.cpp
 * @author Rohan Singh (rohansingh.apjss@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2021-01-26
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "ExtractFeatures.hpp"

namespace object_detector {

ExtractFeatures::ExtractFeatures(ros::NodeHandle& nh) {
    nodeHandle = nh;

    nh.param<double>("/lambda", lambda, 90);
    lambda = lambda * Pi/180;
    nh.param<double>("/abdRatio", abdRatio, 1);
    nh.param<double>("/minSegPts", minSegPts, 5);
    nh.param<double>("/LM/lambdaIni", LM_lambdaIni, 0.001);
    nh.param<int>("/fitMethod", fitMethod, 0);
    nh.param<double>("/maxVelX", maxVelX, 0);
    nh.param<double>("/maxVelY", maxVelY, 0);
    nh.param<std::string>("/laserScanTopic", laserScanTopic, "/laser_horizontal_front");

    scanSub = nodeHandle.subscribe<sensor_msgs::LaserScan>(laserScanTopic, 100, &ExtractFeatures::ScanCallback, this);

    circleOdomSub = nodeHandle.subscribe<nav_msgs::Odometry>("/circleOdom", 100, &ExtractFeatures::OdomCallback, this);

    // detectedCircleScanPub = nodeHandle.advertise<sensor_msgs::LaserScan>("/detectedCircleScan", 100);
    circlePub = nodeHandle.advertise<object_detector::Circle>("/detectedCircle", 100);
}

void ExtractFeatures::ScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    currScan = PreProcessScan(msg);

    currSegments = ExtractSegments(currScan);

    double minDiff = msg->range_max*1000;
    for (auto segment : currSegments) {
        object_detector::Circle circle = DetectCircle(segment, fitMethod);
        if (circle.radius > 0.3 || circle.radius < 0.05) {
            continue;
        } else {
            currCircles.push_back(circle);
        }
    }
    if (currCircles.size() > 0) {
        currCircle = currCircles[0];
        for (auto circle : currCircles) {
            if (std::abs(circle.radius - 0.14) < minDiff) {
                currCircle = circle;
                minDiff = std::abs(circle.radius - 0.14);
            }
        }
        if (std::abs(currCircle.radius - 0.14) < 0.1) {
            currCircle.header = msg->header;
            circlePub.publish(currCircle); 
        }
    }
    currCircles.clear();
}

void ExtractFeatures::OdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    circleOdom = *msg;
}

std::vector<object_detector::Point> ExtractFeatures::PreProcessScan(const sensor_msgs::LaserScan::ConstPtr& msg) {
    std::vector<Point> scan;
    double currAng = msg->angle_min;
    for (int i = 0; i < msg->ranges.size(); i++) {
        if (msg->ranges[i] > msg->range_max || msg->ranges[i] < msg->range_min) {
            currAng += msg->angle_increment;
        } else {
            object_detector::Point pt;
            pt.header = msg->header;
            pt.x = msg->ranges[i] * cos(currAng);
            pt.y = msg->ranges[i] * sin(currAng);
            pt.theta = currAng;
            pt.range = msg->ranges[i];
            scan.push_back(pt);
            currAng += msg->angle_increment;
        }
    }
    return scan;
}

std::vector<std::vector<object_detector::Point>> ExtractFeatures::ExtractSegments(std::vector<object_detector::Point>& scan) {
    std::vector<std::vector<object_detector::Point>> segments;
    int idx = 0, segIdx = 0;
    std::vector<object_detector::Point> segment;

    while (idx < scan.size()) {

        if (idx == segIdx) {
            segment.push_back(scan[idx]);
            idx++;
        } else {
            // Adaptive Breakpoint Detection
            double dtheta = scan[idx].theta - scan[idx-1].theta;
            // double dMax = abdRatio * std::min(scan[idx-1].range, scan[idx].range) * (sin(dtheta) / sin(lambda - dtheta);
            double dMax = abdRatio * scan[idx-1].range * (sin(dtheta) / sin(lambda - dtheta));

            if (CalcAbsDistance(scan[idx-1], scan[idx]) < dMax) {
                segment.push_back(scan[idx]);
                idx++;
                if (idx == scan.size()) {
                    if (segment.size() > minSegPts) {
                        segments.push_back(segment);
                    }
                    break;
                }
            } else {
                segIdx = idx;
                if (segment.size() > minSegPts) {
                    segments.push_back(segment);
                }
                segment.clear();
            }
        }
    }
    return segments;
}

object_detector::Circle ExtractFeatures::DetectCircle(std::vector<object_detector::Point>& segment, int method) {
    std::vector<double> X;
    std::vector<double> Y;
    for (auto p : segment) {
        X.push_back(p.x);
        Y.push_back(p.y);
    }
    Data data(segment.size(), &X[0], &Y[0]);
    CircleForFit c;

    if (method == 0) {
        c = CircleFitByHyper(data);
    } else if (method == 1) {
        CircleForFit cIni;
        cIni.a = Mean(X);
        cIni.b = Mean(Y);
        cIni.r = 0.140;

        if (CircleFitByLevenbergMarquardtFull(data, cIni, LM_lambdaIni, c) != 0) {
            c.a = 0;
            c.b = 0;
            c.r = 0;
        }
    }

    object_detector::Circle circle;
    circle.x = c.a;
    circle.y = c.b;
    circle.radius = c.r;
    circle.segment = segment;

    return circle;
}

double ExtractFeatures::CalcAbsDistance(object_detector::Point& p1, object_detector::Point& p2) {
    return sqrt((p1.x - p2.x)*(p1.x - p2.x) + (p1.y - p2.y)*(p1.y - p2.y));
}

double ExtractFeatures::Mean(std::vector<double> vec) {
    double sum = 0;
    for (auto v : vec) {
        sum += v;
    }
    return sum/vec.size();
}

}
