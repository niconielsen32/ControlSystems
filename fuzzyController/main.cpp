#include <iostream>
#include <opencv2/opencv.hpp>
#include <typeinfo>
#include <cmath>
#include <random>
#include <time.h>

#include "lidarsensor.h"
#include "callBackFunctions.h"
#include "fuzzycontroller.h"
#include "pose.h"


const bool showLidar = false;
const bool showPose = false;
const bool showCamera = false;
const bool showStat = false;

boost::mutex mutex;



int main(int _argc, char **_argv) {

    // Load gazebo
    gazebo::client::setup(_argc, _argv);

    // Create our node for communication
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();

    // Listen to Gazebo topics
    gazebo::transport::SubscriberPtr statSubscriber;
    gazebo::transport::SubscriberPtr poseSubscriber;
    gazebo::transport::SubscriberPtr cameraSubscriber;
    gazebo::transport::SubscriberPtr lidarSubscriber;

    //Display the different types of data/information/frames
    if(showStat){
     statSubscriber = node->Subscribe("~/world_stats", statCallback);
    }

    if(showCamera){
    cameraSubscriber = node->Subscribe("~/pioneer2dx/camera/link/camera/image", cameraCallback);
    }

    if(showPose){
    poseSubscriber = node->Subscribe("~/pose/info", poseCallback);
    }

    if(showLidar){
    lidarSubscriber = node->Subscribe("~/pioneer2dx/hokuyo/link/laser/scan", lidarCallback);
    }


    // Publish to the robot vel_cmd topic
    gazebo::transport::PublisherPtr movementPublisher = node->Advertise<gazebo::msgs::Pose>("~/pioneer2dx/vel_cmd");

    // Publish a reset of the world
    gazebo::transport::PublisherPtr worldPublisher = node->Advertise<gazebo::msgs::WorldControl>("~/world_control");


    gazebo::msgs::WorldControl controlMessage;
    controlMessage.mutable_reset()->set_all(true);
    worldPublisher->WaitForConnection();
    worldPublisher->Publish(controlMessage);


    // Create an object for marble detection
    //MarbleDetection marble;
    //marble.runMarbleDetection();

    // Create an object for lidar sensor
    lidarSensor lidar;
    lidar.runLidarSensor();

    std::tuple<float,float,float> ranges;
    float distRight = 10.0;
    float distFront = 10.0;
    float distLeft = 10.0;
    float distMinCon = 1.5;

    // Create an object and set up both fuzzy controllers
    fuzzyController controller;
    controller.setupFuzzyController();
    fuzzyController controllerGoal;
    controllerGoal.setupFuzzyControllerGoal();

    // Create an object to get poseCallback info for robot
    pose Pose;
    Pose.runPose();

    // keys to control robot
//    const int key_left = 81;
//    const int key_up = 82;
//    const int key_down = 84;
//    const int key_right = 83;
//    const int key_esc = 27;

    // initialize robot states
    bool emergencyStop = false;
    bool goalReached = false;
    bool terminate = false;

    // set speed, direction and angle calculation for the robot
    float speed = 0.0;
    double dir = 0.0;
    double angleRobToPoint = 0.0;
    double angleYaw = 0.0;
    double inputAngle = 0.0;


    // Test right half of the map
    //std::vector<cv::Point2d> goalPos {cv::Point2d(22.0, 6.0), cv::Point2d(34.0, 20.0), cv::Point2d(33.0, -12.0), cv::Point2d(18.0, -7.0), cv::Point2d(16.0, -21.0), cv::Point2d(-4.0, -21.0)};

    // Test left half of the map
    //std::vector<cv::Point2d> goalPos {cv::Point2d(-17.0, -2.0), cv::Point2d(-37.0, -6.0), cv::Point2d(-16.0, 0.0), cv::Point2d(-9.0, 16.0), cv::Point2d(0.0, 8.0), cv::Point2d(-26.0, 16.0), cv::Point2d(-37.0, 23.0)};
    std::vector<cv::Point2d> goalPos {cv::Point2d(-17.0, -2.0), cv::Point2d(-14.0, 18.0)};
    std::vector<cv::Point2d> goalPosTemp = goalPos;

    // To visualize the paths taken by the robot from start to goal
    cv::Mat finalPath = cv::imread("floor_plan.png");
    std::vector<cv::Point2d> robToGoalPath;
    int counter = 0;
    srand(time(NULL));

    // Infinite Loop
    while (!goalReached) {

        // get lidar minimum ranges from the three different directions
        ranges = lidar.getMinRanges();
        distRight = std::get<0>(ranges);
        distFront = std::get<1>(ranges);
        distLeft = std::get<2>(ranges);

        // Distance to marble
        //marble.distanceToMarbleOld(&lidar);
        //marble.distanceToMarble(distFront);


        // Fuzzy controller angle calculation for go to goal point
        angleRobToPoint = std::atan2(goalPosTemp[0].y - Pose.getPosY(), goalPosTemp[0].x - Pose.getPosX());
        angleYaw = Pose.getYaw();

        if ((angleRobToPoint < -1.57 && angleYaw < 0 && angleYaw > -1.57) || (angleRobToPoint > 0.0 && angleRobToPoint < 1.57 && angleYaw < -1.57)) {
            inputAngle = angleRobToPoint - std::abs(angleYaw);
        } else if ((angleRobToPoint < -1.57 && angleYaw > 1.57) || (angleRobToPoint > 1.57  && angleYaw < 0.0)) {
            inputAngle = angleYaw - angleRobToPoint;
        } else {
            inputAngle = angleRobToPoint - angleYaw;
        }

        std::cout << "posX: " << Pose.getPosX() << " posY: " << Pose.getPosY() << " yaw: " << angleYaw << std::endl;
        std::cout << "angle: " << angleRobToPoint << std::endl;
        std::cout << "angle-yaw: " << inputAngle << std::endl;
        std::cout << "goalPos size: " << goalPosTemp.size() << std::endl;


        gazebo::common::Time::MSleep(10);

        int key = cv::waitKey(1);

        // if "q" pressed - terminate program
//        if (key == key_esc){
//          break;
//        }

        // robot controller with arrow keys
//        if ((key == key_up))
//          speed += 0.05;
//        else if ((key == key_down))
//          speed -= 0.05;
//        else if ((key == key_right) && (dir <= 0.4f))
//          dir += 0.05;
//        else if ((key == key_left) && (dir >= -0.4f))
//          dir -= 0.05;
//        else {
//          // slow down
//          //      speed *= 0.1;
//               //dir *= 0.1;
//        }

        // check if goal pos is reached
        if (( goalPosTemp[0].x == int(Pose.getPosX()) && goalPosTemp[0].y == int(Pose.getPosY()) )|| terminate) {
            std::cout << " sub goal reached" << std::endl;

            // erase the reached goal and go to next goal
            goalPosTemp.erase(goalPosTemp.begin());

            // To visualize the paths taken by the robot from start to subGoal
            cv::Scalar color(rand()%255, rand()%255, rand()%255);
            for(size_t i = 0; i < robToGoalPath.size(); i++) {
                robToGoalPath[i].x = (robToGoalPath[i].x*1.38 + finalPath.cols/2);
                robToGoalPath[i].y *= -1;
                robToGoalPath[i].y = (robToGoalPath[i].y*1.48 + finalPath.rows/2);
                cv::circle(finalPath, robToGoalPath[i], 0, color, 1);
            }

            robToGoalPath.clear();

            if (goalPosTemp.size() == 0 || terminate) {
                goalReached = true;
            }
        }

        // different statea for the robot to be in
        if(key == 's'){
            emergencyStop = true;
        }

        if(key == 'r'){
            emergencyStop = false;
        }

        if(key == 't') {
            terminate = true;
        }

        // Choose which fuzzy controller to run
        if(distRight < distMinCon || distFront < distMinCon || distLeft < distMinCon){
            controller.runFuzzyController(distFront, distLeft, distRight);
        } else {
            controllerGoal.runFuzzyControllerGoal(inputAngle);
        }

        // robot control
        if(emergencyStop){
            speed = 0.0;
            dir = 0.0;
            std::cout << "emergency stop" << std::endl;
        } else if (goalReached) {
            dir = 0.0;
            speed = 0.0;
            std::cout << "end goal reached" << std::endl;
        } else if((distRight < distMinCon && distRight != 0) || (distFront <= distMinCon && distFront != 0) || (distLeft < distMinCon && distLeft != 0)){
            dir = controller.getOutputDirection();
            speed = controller.getOutputVelocity();
            std::cout << "controller obstacle avoidance" << std::endl;
        } else if((distRight > distMinCon && distRight != 0) || (distFront > distMinCon && distFront != 0) || (distLeft > distMinCon && distLeft != 0)){
            dir = controllerGoal.getOutputDirectionGoal();
            speed = controllerGoal.getOutputVelocityGoal();
            std::cout << "controller goal" << std::endl;
        }

        std::cout << "speed: " << speed << std::endl;
        std::cout << "dir: " << dir << std::endl;


        // Generate a pose
        ignition::math::Pose3d pose(double(speed), 0, 0, 0, 0, double(dir));

        // gets robot position to visualize taken paths
        counter++;
        if (counter == 20) {
            robToGoalPath.push_back(cv::Point2d(Pose.getPosX(), Pose.getPosY()));
            counter = 0;
        }

        // Convert to a pose message
        gazebo::msgs::Pose msg;
        gazebo::msgs::Set(&msg, pose);
        movementPublisher->Publish(msg);

    }

    // visualization of marbles from given map
    std::vector<cv::Point2d> marblesPos {cv::Point2d(-13.20, -6.43), cv::Point2d(-31.58, -4.01), cv::Point2d(11.62,-18.21), cv::Point2d(-17.30, 18.32), cv::Point2d(-29.62, 10.65),
                                         cv::Point2d(8.74, 11.85), cv::Point2d(24.73, 14.43), cv::Point2d(-22.45, -6.26), cv::Point2d(-23.57, 8.09), cv::Point2d(11.7305, -3.1118),
                                         cv::Point2d(16.28, -13.15), cv::Point2d(26.70, -10.66), cv::Point2d(25.73, 9.65), cv::Point2d(8.20, -3.52), cv::Point2d(30.94, -9.27),
                                         cv::Point2d(25.76, -0.12), cv::Point2d(19.22, 14.21), cv::Point2d(29.92, 21.98), cv::Point2d(2.82, 16.63), cv::Point2d(-15.69, -4.22)};

    for(size_t i = 0; i < marblesPos.size(); i++) {
        marblesPos[i].x = (marblesPos[i].x*1.38 + finalPath.cols/2);
        marblesPos[i].y *= -1;
        marblesPos[i].y = (marblesPos[i].y*1.48 + finalPath.rows/2);
        cv::circle(finalPath, marblesPos[i], 1, cv::Scalar(128,128,128), -1);
    }

    // visualize start-, sub- and endPos of robot path
    for(size_t i = 0; i < goalPos.size(); i++) {
        goalPos[i].x = (goalPos[i].x*1.38 + finalPath.cols/2);
        goalPos[i].y *= -1;
        goalPos[i].y = (goalPos[i].y*1.48 + finalPath.rows/2);
        cv::circle(finalPath, goalPos[i], 1, cv::Scalar(255,0,0), -1);
    }
    cv::circle(finalPath, cv::Point2d(finalPath.cols/2, finalPath.rows/2), 1, cv::Scalar(0,255,0), -1);
    cv::circle(finalPath, goalPos[goalPos.size()-1], 1, cv::Scalar(0,0,255), -1);

    cv::namedWindow("finalPath");
    cv::resize(finalPath, finalPath, cv::Size(finalPath.cols*5, finalPath.rows*5), 0, 0, cv::INTER_NEAREST);
    cv::imshow("finalPath", finalPath);
    cv::imwrite("finalPath.png", finalPath);
    cv::waitKey(0);

    // Make sure to shut everything down.
    gazebo::client::shutdown();
}
