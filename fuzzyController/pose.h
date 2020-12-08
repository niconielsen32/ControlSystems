#ifndef POSE_H
#define POSE_H

#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include <opencv2/opencv.hpp>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

#include <bits/stdc++.h>

extern boost::mutex mutex;

class pose
{
public:
    pose();

    void runPose();

    double getPosX() {return posX;};
    double getPosY() {return posY;};
    double getYaw() {return yaw;};


private:
    void poseCallback(ConstPosesStampedPtr &_msg);

    gazebo::transport::Node nodePose;
    gazebo::transport::SubscriberPtr subPose;

    double posX = 0.0;
    double posY = 0.0;
    double yaw = 0.0;

};

#endif // POSE_H
