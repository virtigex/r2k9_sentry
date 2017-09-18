// R2K9 controller code

#include <cstdio>
#include <memory>
#include <string>
#include <sstream>
#include <vector>
#include <iomanip>
#include <cmath>
#include <csignal>
#include <mutex>
#include <ctime>
#include <chrono>

// ROS

#include <ros/ros.h>
#include <ros/types.h>
#include <ros/message.h>

#include <rosbag/bag.h>
#include <rosbag/recorder.h>
#include <rosbag/exceptions.h>

//
// ROS messages
//
#include "r2k9_sentry/TrainingReady.h"

#include "boost/program_options.hpp"
#include "boost/filesystem.hpp"
#include "boost/thread/mutex.hpp"

#include <realsense_person/PersonDetection.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>

#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/MotorPower.h>

//
// ROS paths
//
#define PARAM_RECORD ("/r2k9/record")
#define PARAM_SAY_STARTUP ("/r2k9/say/startup")
#define PARAM_SAY_SHUTDOWN ("/r2k9/say/shutdown")
#define PARAM_SAY_TRACK_START ("/r2k9/say/track_start")
#define PARAM_SAY_TRACK_STOP ("/r2k9/say/track_stop")

#define TOPIC_PERSON_DETECTION_DATA ("/camera/person/detection_data")
#define TOPIC_PERSON_DETECTION_IMAGE ("/camera/person/detection_image")
#define TOPIC_POINTCLOUD ("/camera/depth/points")
#define TOPIC_IMAGE ("/camera/color/image_raw/compressed")

#define TOPIC_VELOCITY "/mobile_base/commands/velocity"
#define TOPIC_MOTOR ("/mobile_base/commands/motor_power")
#define TOPIC_TRAINING ("/r2k9/training")

//
// DEFAULT PARAMETERS
//

#define DEFAULT_X_TRIGGER (0.02)
#define DEFAULT_ANGULAR_VELOCITY (0.40)

#include "utils.h"

// UTILITY FUNCTIONS

void SayParameterText(std::string param_text)
{
  std::string speakCommand("/usr/bin/spd-say");
  if (!boost::filesystem::exists(speakCommand))
    return;
  std::string text;
  if (ros::param::get(param_text, text) && text.length() > 0) {
    std::system((speakCommand + " \"" + text +"\"").c_str());
  }
}

//
// R2K9 Sentry
//
class R2k9Control
{
public:
  R2k9Control();
  ~R2k9Control();
private:
  void personCallback(const ros::MessageEvent<realsense_person::PersonDetection const>& event);
  void personImageCallback(const ros::MessageEvent<sensor_msgs::Image const>& event);
  void pointCloudCallback(const ros::MessageEvent<sensor_msgs::PointCloud2 const>& event);
  void imageCallback(const ros::MessageEvent<sensor_msgs::CompressedImage const>& event);

  void move(double vel_linear, double vel_angular);
  bool recordingStart(ros::Time start);
  void recordingStop();

  ros::NodeHandle  nh_;
  ros::Subscriber subPerson_;
  ros::Subscriber subPersonImage_;
  ros::Subscriber subPointCloud_;
  ros::Subscriber subImage_;
  ros::Publisher vel_pub_;
  ros::Publisher motor_pub_;
  ros::Publisher training_pub_;
  rosbag::Bag bag_;
  bool person_present_;
  bool stationary_;
  bool recording_;
  std::mutex writeMutex_;
  std::string recordFileStaging_;
  std::string recordFile_;
};

R2k9Control::R2k9Control()
{
  subPerson_ = nh_.subscribe(TOPIC_PERSON_DETECTION_DATA, 1000, &R2k9Control::personCallback, this);
  subPersonImage_ = nh_.subscribe(TOPIC_PERSON_DETECTION_IMAGE, 1000, &R2k9Control::personImageCallback, this);
  subPointCloud_ = nh_.subscribe(TOPIC_POINTCLOUD, 1000, &R2k9Control::pointCloudCallback, this);
  subImage_ = nh_.subscribe(TOPIC_IMAGE, 1000, &R2k9Control::imageCallback, this);
  vel_pub_ = nh_.advertise<geometry_msgs::Twist>(TOPIC_VELOCITY, 1);
  motor_pub_ = nh_.advertise<kobuki_msgs::MotorPower>(TOPIC_MOTOR, 1);
  training_pub_ = nh_.advertise<r2k9::TrainingReady>(TOPIC_TRAINING, 1);
  person_present_ = false;
  stationary_ = true;
  recording_ = false;
  SayParameterText(PARAM_SAY_STARTUP);
}

R2k9Control::~R2k9Control()
{
  SayParameterText(PARAM_SAY_SHUTDOWN);
  std::cout << "cleaning up after r2k9" << std::endl;
  if (recording_) {
    bag_.close();
  }
  if (boost::filesystem::exists(recordFileStaging_)) {
    boost::filesystem::remove(recordFileStaging_);
  }
}

void R2k9Control::move(double vel_linear, double vel_angular)
{
  if (stationary_) {
    stationary_ = false;
    kobuki_msgs::MotorPower pwr;
    pwr.state = pwr.ON;
    motor_pub_.publish(pwr);
  }
  double minmove = 0.001;
  bool still = fabs(vel_linear) < minmove && fabs(vel_angular) < minmove;
  geometry_msgs::Twist vel;
  vel.angular.x = 0.0;
  vel.angular.y = 0.0;
  vel.angular.z = vel_angular;
  vel.linear.x = vel_linear;
  vel.linear.y = 0.0;
  vel.linear.z = 0.0;
  if (!still) {
    ROS_INFO_STREAM("move " << vel_linear << ", " <<  vel_angular);
  }

  vel_pub_.publish(vel);
}

void R2k9Control::personCallback(const ros::MessageEvent<realsense_person::PersonDetection const>& event)
{
  bool moved = false;
  if (event.getMessage()->detected_person_count > 0) {
    ROS_INFO_STREAM("persons detected " << event.getMessage()->detected_person_count);
    realsense_person::Person person = event.getMessage()->persons[0]; // pick first person
    int32_t wconf = person.center_of_mass.world_confidence;
    int32_t iconf = person.center_of_mass.image_confidence;
    geometry_msgs::Point cworld = person.center_of_mass.world;

    ROS_INFO_STREAM(std::setprecision(2) << std::fixed
      << "position=(" <<  cworld.x << "," << cworld.y << "," << cworld.z << ")" << " conf=" << wconf);
    if (cworld.x < -DEFAULT_X_TRIGGER) {
      move(0.0, DEFAULT_ANGULAR_VELOCITY);
      moved = true;
    }
    if (cworld.x > DEFAULT_X_TRIGGER) {
      move(0.0, -DEFAULT_ANGULAR_VELOCITY);
      moved = true;
    }
    if (!person_present_) {
      SayParameterText(PARAM_SAY_TRACK_START);
      recordingStart(event.getMessage()->header.stamp);
      person_present_ = true;
    }
  }
  else {
    if (person_present_) {
      SayParameterText(PARAM_SAY_TRACK_STOP);
      ROS_INFO_STREAM("no persons detected");
      recordingStop();
    }
    person_present_ = false;
  }
  if (!moved) {
    move(0.0, 0.0);
  }

  std::lock_guard<std::mutex> lock(writeMutex_);
  if (recording_) {
    bag_.write(TOPIC_PERSON_DETECTION_DATA, event);
  }
}
void R2k9Control::personImageCallback(const ros::MessageEvent<sensor_msgs::Image const>& event)
{
  std::lock_guard<std::mutex> lock(writeMutex_);
  if (recording_) {
    bag_.write(TOPIC_PERSON_DETECTION_IMAGE, event);
  }
}

void R2k9Control::imageCallback(const ros::MessageEvent<sensor_msgs::CompressedImage const>& event)
{
  std::lock_guard<std::mutex> lock(writeMutex_);
  if (recording_) {
    bag_.write(TOPIC_IMAGE, event);
  }
}

void R2k9Control::pointCloudCallback(const ros::MessageEvent<sensor_msgs::PointCloud2 const>& event)
{
  std::lock_guard<std::mutex> lock(writeMutex_);
  if (recording_) {
    bag_.write(TOPIC_POINTCLOUD, event);
  }
}

bool R2k9Control::recordingStart(ros::Time start)
{
  std::lock_guard<std::mutex> lock(writeMutex_);
  if (recording_)
    return true;
  std::string recordDir;
  if (ros::param::get(PARAM_RECORD, recordDir)) {
    // check it exists
    boost::filesystem::path p(recordDir);
    if (boost::filesystem::is_directory(p)) {
      std::time_t t = start.toSec();
      auto tm = *std::localtime(&t);

      std::stringstream bagName;
      bagName << recordDir << std::put_time(&tm, "/%Y-%m-%d-%H-%M-%S.bag");
      recordFile_ = bagName.str();

      std::stringstream stagingFile;
      stagingFile << recordDir << "/stage-" << ::getpid() << ".bag";

      recordFileStaging_ = stagingFile.str();

      ROS_INFO_STREAM("opening: " << recordFileStaging_);
      bag_.open(recordFileStaging_, rosbag::bagmode::Write);
      recording_ = true;
      ROS_INFO_STREAM("recording started: " << bagName.str());
    }
    else {
      ROS_INFO_STREAM("cannot access training folder " << recordDir.c_str());
    }
  }
  return recording_;
}

void R2k9Control::recordingStop()
{
  std::lock_guard<std::mutex> lock(writeMutex_);
  if (recording_) {
    recording_ = false;
    bag_.close();
    std::rename(recordFileStaging_.c_str(), recordFile_.c_str());
    r2k9::TrainingReady msg;

    msg.location = recordFile_;
    msg.host = getIPV4Address();
    training_pub_.publish(msg);
    ROS_INFO_STREAM("training ready: " << recordFile_);

    recordFile_ = "";
    recordFileStaging_ = "";
  }
}

using namespace std;

void int_handler(int x)
{
  ROS_INFO("shutting down");
  ros::shutdown();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "r2k9_sentry");

  {
    R2k9Control r2k9Control;

    signal(SIGINT, int_handler);


    ROS_INFO("r2k9 active");
    ros::MultiThreadedSpinner spinner(4);
    spinner.spin(); 
    cout << "shutdown complete" << endl;
  }

  return 0;
}
