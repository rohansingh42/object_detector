/**
 * @file TrackFeatureNode.cpp
 * @author Rohan Singh (rohansingh.apjss@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2021-01-26
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "TrackFeature.hpp"

int main(int argc, char** argv) {

    ros::init(argc, argv, "track_feature");
    ros::NodeHandle nh("~");

    object_detector::TrackFeature tf(nh);
    ros::spin();

  return 0;
}