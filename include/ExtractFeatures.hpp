/**
 * @file ExtractFeatures.hpp
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

class ExtractFeatures {
  private:
    ros::NodeHandle nodeHandle;
    ros::Subscriber scanSub, circleOdomSub;
    ros::Publisher circlePub, detectedCircleScanPub;

    std::vector<object_detector::Point> currScan;
    std::vector<std::vector<object_detector::Point>> currSegments;
    std::vector<object_detector::Circle> currCircles;
    object_detector::Circle currCircle;
    object_detector::Circle prevCircle;
    sensor_msgs::LaserScan detectedScan;
    nav_msgs::Odometry circleOdom;

    double lambda;
    double abdRatio;
    double minSegPts;
    double LM_lambdaIni;
    int fitMethod;    //0 - hypercircle, 1 - LM
    double maxVelX;
    double maxVelY;

  public:

    /**
     * @brief Construct a new Extract Features object
     * 
     * @param nh 
     */
    ExtractFeatures(ros::NodeHandle& nh);

    /**
     * @brief Laser Scan Callback
     * 
     * @param msg 
     */
    void ScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

    /**
     * @brief Converts Laser Scan into vector of custom datatype(Point)
     * 
     * @param msg 
     * @return std::vector<object_detector::Point> 
     */
    std::vector<object_detector::Point> PreProcessScan(const sensor_msgs::LaserScan::ConstPtr& msg);

    /**
     * @brief Extracts segments from the scan based on Adaptive BreakPoint Detection
     * 
     * @param scan 
     * @return std::vector<std::vector<object_detector::Point>> 
     */
    std::vector<std::vector<object_detector::Point>> ExtractSegments(std::vector<object_detector::Point>& scan);

    /**
     * @brief Fit a circle in a given set of points
     * 
     * @param segment Vector of Circle
     * @param method Method to be used to fit a circle
     * @return object_detector::Circle 
     */
    object_detector::Circle DetectCircle(std::vector<object_detector::Point>& segment, int method);

    /**
     * @brief Calculate Absolute distance between 2 points
     * 
     * @param p1 
     * @param p2 
     * @return double 
     */
    double CalcAbsDistance(object_detector::Point& p1, object_detector::Point& p2);

    /**
     * @brief Calculate mean of given set of points
     * 
     * @param vec vector of doubles
     * @return double 
     */
    double Mean(std::vector<double> vec);

    /**
     * @brief Callback for Circle Odometry
     * 
     * @param msg 
     */
    void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg);
};

}