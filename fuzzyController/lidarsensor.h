#ifndef LIDARSENSOR_H
#define LIDARSENSOR_H
#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include <opencv2/opencv.hpp>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

#include <bits/stdc++.h>

extern boost::mutex mutex;


class lidarSensor
{

public:
    lidarSensor();

    void runLidarSensor();

    std::tuple<float, float, float> getMinRanges() { return minRanges; }
    void setMinRanges();

    std::vector<float> get_ranges(){ return rangesTest; }

    float getClosestObjectFront(){ return closestObjectFront; }


private:

    std::tuple<float, float, float> minRanges;

    float obstacleCollisionThreshold = 10;

    const int rightRange = 95, frontRange = 105, leftRange = 200;

    // Vectors with distances for each section
    float closestObjectLeft = 0;
    float closestObjectFront = 0;
    float closestObjectRight = 0;

    std::vector<float> rightRanges = {};
    std::vector<float> frontRanges = {};
    std::vector<float> leftRanges = {};

    std::vector<float> rangesTest = {};


    void lidarCallback(ConstLaserScanStampedPtr &msg);

    gazebo::transport::Node nodeLidar;
    gazebo::transport::SubscriberPtr subLidar;



};

#endif // LIDARSENSOR_H
