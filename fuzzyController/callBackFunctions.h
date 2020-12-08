#ifndef CALLBACKFUNCTIONS_H
#define CALLBACKFUNCTIONS_H

extern boost::mutex mutex;

void statCallback(ConstWorldStatisticsPtr &_msg) {
  (void)_msg;
  // Dump the message contents to stdout.
  //  std::cout << _msg->DebugString();
  //  std::cout << std::flush;
}


void poseCallback(ConstPosesStampedPtr &_msg) {
  // Dump the message contents to stdout.
  //  std::cout << _msg->DebugString();

  // Pose for the robot
  for (int i = 0; i < _msg->pose_size(); i++) {
    if (_msg->pose(i).name() == "pioneer2dx") {

     std::cout << std::setprecision(2) << std::fixed << std::setw(6)
                << _msg->pose(i).position().x() << std::setw(6)
                << _msg->pose(i).position().y() << std::setw(6)
                << _msg->pose(i).position().z() << std::setw(6)
                << _msg->pose(i).orientation().w() << std::setw(6)
                << _msg->pose(i).orientation().x() << std::setw(6)
                << _msg->pose(i).orientation().y() << std::setw(6)
                << _msg->pose(i).orientation().z() << std::endl;
    }
  }
}


void cameraCallback(ConstImageStampedPtr &msg){

  mutex.lock();

  static bool initialized;
  if (!initialized)
  {
      initialized = true;
      cv::namedWindow("camera", cv::WINDOW_AUTOSIZE);
  }
  // Convert msg image to a Mat object
  std::size_t width = msg->image().width();
  std::size_t height = msg->image().height();
  const char *data = msg->image().data().c_str();
  cv::Mat im(int(height), int(width), CV_8UC3, const_cast<char *>(data));

  // convert color and smooth image to reduce noise
  cv::cvtColor(im, im, cv::COLOR_RGB2BGR);
  //cv::cvtColor(im, im, CV_RGB2GRAY);
  //cv::GaussianBlur( im, im, Size(9, 9), 2, 2 );

  // Show camera frame
  cv::imshow("camera", im);
  mutex.unlock();

}



void lidarCallback(ConstLaserScanStampedPtr &msg) {

  mutex.lock();

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
  // go through all the lidar points - get distance - draw points and lines for displaying lidar data
  for (int i = 0; i < 80; i++) {
    // angle for the lidar point
    float angle = angle_min + i * angle_increment;
    // distance from robot to where the lidar point hit an obstacle
    float range = std::min(float(msg->scan().ranges(i)), range_max);
    //    double intensity = msg->scan().intensities(i);
    if(range <= 1){
        cv::Point2f startpt(200.5f + range_min * px_per_m * std::cos(angle),
                            200.5f - range_min * px_per_m * std::sin(angle));
        cv::Point2f endpt(200.5f + range * px_per_m * std::cos(angle),
                          200.5f - range * px_per_m * std::sin(angle));
        cv::line(im, startpt * 16, endpt * 16, cv::Scalar(0, 255, 0, 255), 1,
                 cv::LINE_AA, 4);
    }
  }

  for (int i = 80; i < 120; i++) {
    // angle for the lidar point
    float angle = angle_min + i * angle_increment;
    // distance from robot to where the lidar point hit an obstacle
    float range = std::min(float(msg->scan().ranges(i)), range_max);
    //    double intensity = msg->scan().intensities(i);
    if(range <= 1){
        cv::Point2f startpt(200.5f + range_min * px_per_m * std::cos(angle),
                            200.5f - range_min * px_per_m * std::sin(angle));
        cv::Point2f endpt(200.5f + range * px_per_m * std::cos(angle),
                          200.5f - range * px_per_m * std::sin(angle));
        cv::line(im, startpt * 16, endpt * 16, cv::Scalar(0, 255, 0, 255), 1,
                 cv::LINE_AA, 4);
    }
  }

  for (int i = 120; i < nranges; i++) {
    // angle for the lidar point
    float angle = angle_min + i * angle_increment;
    // distance from robot to where the lidar point hit an obstacle
    float range = std::min(float(msg->scan().ranges(i)), range_max);
    //    double intensity = msg->scan().intensities(i);
    if(range <= 1){
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


  // Show the lidar data
  cv::moveWindow("Lidar", 1500, 350);
  cv::imshow("Lidar", im);
  mutex.unlock();
}

#endif // CALLBACKFUNCTIONS_H
