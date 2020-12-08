#include "pose.h"

pose::pose()
{

}

void pose::runPose()
{
    gazebo::transport::NodePtr nodePose(new gazebo::transport::Node());
    nodePose->Init();
    subPose = nodePose->Subscribe("~/pose/info", &pose::poseCallback, this);
}

void pose::poseCallback(ConstPosesStampedPtr &_msg) {
  // Pose for the robot
  for (int i = 0; i < _msg->pose_size(); i++) {
    if (_msg->pose(i).name() == "pioneer2dx") {
        posX = _msg->pose(i).position().x();
        posY = _msg->pose(i).position().y();

        double qW = _msg->pose(i).orientation().w();
        double qX = _msg->pose(i).orientation().x();
        double qY = _msg->pose(i).orientation().y();
        double qZ = _msg->pose(i).orientation().z();

        double siny_cosp = 2*(qW * qZ - qX * qY);
        double cosy_cosp = 1 - 2*(qY * qY + qZ * qZ);
        yaw = std::atan2(siny_cosp, cosy_cosp);
    }
  }
}


