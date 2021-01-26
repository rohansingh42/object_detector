/**
 * @file ExtractFeaturesNode.cpp
 * @author Rohan Singh (rohansingh.apjss@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2021-01-26
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "ExtractFeatures.hpp"

int main(int argc, char** argv) {

    ros::init(argc, argv, "extract_features");
    ros::NodeHandle nh("~");

    object_detector::ExtractFeatures ef(nh);
    ros::spin();

  return 0;
}