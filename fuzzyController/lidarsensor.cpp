    #include "lidarsensor.h"

lidarSensor::lidarSensor(){
}

void lidarSensor::runLidarSensor(){
    gazebo::transport::NodePtr nodeLidar(new gazebo::transport::Node());
    nodeLidar->Init();
    subLidar = nodeLidar->Subscribe("~/pioneer2dx/hokuyo/link/laser/scan", &lidarSensor::lidarCallback, this);

}


void lidarSensor::setMinRanges(){

    closestObjectLeft = *min_element(leftRanges.begin(), leftRanges.end());
    closestObjectFront = *min_element(frontRanges.begin(), frontRanges.end());
    closestObjectRight = *min_element(rightRanges.begin(), rightRanges.end());;

    minRanges = {closestObjectRight, closestObjectFront, closestObjectLeft};
    rightRanges.clear();
    frontRanges.clear();
    leftRanges.clear();
}


void lidarSensor::lidarCallback(ConstLaserScanStampedPtr &msg) {


    static bool initialized;
    if(!initialized){
    initialized = true;
    cv::namedWindow("Lidar", cv::WINDOW_AUTOSIZE);
    }

    //  std::cout << ">> " << msg->DebugString() << std::endl;
    float angle_min = float(msg->scan().angle_min()); // ~ -130 degree
    //  double angle_max = msg->scan().angle_max();
    float angle_increment = float(msg->scan().angle_step()); // ~ 1.6 degree

    // min and max range for lidar
    float range_min = float(msg->scan().range_min()); // 0.08
    float range_max = float(msg->scan().range_max()); // 10

    int sec = msg->time().sec();
    int nsec = msg->time().nsec();

    // resolution of lidar
    int nranges = msg->scan().ranges_size();
    int nintensities = msg->scan().intensities_size();

    assert(nranges == nintensities);

    int width = 400;
    int height = 400;
    float px_per_m = 200 / range_max;
    cv::Mat im(height, width, CV_8UC3);
    im.setTo(0);

    //RIGHT LIDAR RANGE
    // go through all the lidar points - get distance - draw points and lines for displaying lidar data
    for (int i = 30; i < rightRange; i++) {
    // angle for the lidar point
        float angle = angle_min + i * angle_increment;
        // distance from robot to where the lidar point hit an obstacle
        float range = std::min(float(msg->scan().ranges(i)), range_max);
        //    double intensity = msg->scan().intensities(i);
        rightRanges.push_back(range);
        rangesTest.push_back(range);

        if(range <= obstacleCollisionThreshold){
            cv::Point2f startpt(200.5f + range_min * px_per_m * std::cos(angle),
                                200.5f - range_min * px_per_m * std::sin(angle));
            cv::Point2f endpt(200.5f + range * px_per_m * std::cos(angle),
                              200.5f - range * px_per_m * std::sin(angle));
            cv::line(im, startpt * 16, endpt * 16, cv::Scalar(0, 255, 0, 255), 1,
                     cv::LINE_AA, 4);
        }
    }

    int key = cv::waitKey(1);

    std::vector<float> rangesTest;
    // FRONT LIDAR RANGE
    for (int i = rightRange; i < frontRange; i++) {
        // angle for the lidar point
        float angle = angle_min + i * angle_increment;
        // distance from robot to where the lidar point hit an obstacle
        float range = std::min(float(msg->scan().ranges(i)), range_max);

        frontRanges.push_back(range);
        rangesTest.push_back(range);

        if(key == 'p'){

            std::cout << "Range: " << range << std::endl;
        }

        //    double intensity = msg->scan().intensities(i);
        cv::Point2f startpt(200.5f + range_min * px_per_m * std::cos(angle),
                            200.5f - range_min * px_per_m * std::sin(angle));
        cv::Point2f endpt(200.5f + range * px_per_m * std::cos(angle),
                          200.5f - range * px_per_m * std::sin(angle));
        cv::line(im, startpt * 16, endpt * 16, cv::Scalar(0, 255, 0, 255), 1,
                 cv::LINE_AA, 4);

    }

    //LEFT LIDAR RANGE
    for (int i = frontRange; i < leftRange-30; i++) {
        // angle for the lidar point
        float angle = angle_min + i * angle_increment;
        // distance from robot to where the lidar point hit an obstacle
        float range = std::min(float(msg->scan().ranges(i)), range_max);
        //    double intensity = msg->scan().intensities(i);

        leftRanges.push_back(range);
        rangesTest.push_back(range);

        if(range <= obstacleCollisionThreshold){
            cv::Point2f startpt(200.5f + range_min * px_per_m * std::cos(angle),
                                200.5f - range_min * px_per_m * std::sin(angle));
            cv::Point2f endpt(200.5f + range * px_per_m * std::cos(angle),
                              200.5f - range * px_per_m * std::sin(angle));
            cv::line(im, startpt * 16, endpt * 16, cv::Scalar(0, 255, 0, 255), 1,
                     cv::LINE_AA, 4);
        }
    }

    cv::circle(im, cv::Point(200, 200), 2, cv::Scalar(0, 0, 255));
    cv::putText(im, std::to_string(sec) + ":" + std::to_string(nsec),
              cv::Point(10, 20), cv::FONT_HERSHEY_PLAIN, 1.0,
              cv::Scalar(255, 0, 0));

    setMinRanges();


    mutex.lock();
    // Show the lidar data
    cv::moveWindow("Lidar", 1500, 350);
    //cv::imshow("Lidar", im);
    mutex.unlock();
}
