#ifndef SIMPLE_RECOGNITION_TESTER_H_
#define SIMPLE_RECOGNITION_TESTER_H_

//ROS
#include <nimbus_perception/ClassifyInstance.h>
#include <pcl_ros/point_cloud.h>
#include <rail_manipulation_msgs/SegmentedObjectList.h>
#include <ros/ros.h>
#include <sensor_msgs/point_cloud_conversion.h>

//PCL
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

//C++
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <signal.h>
#include <stdio.h>
#include <termios.h>

#define KEYCODE_Q 0x71
#define KEYCODE_W 0x77
#define KEYCODE_D 0x64

class SimpleRecognitionTester
{

public:

  /**
   * \brief Constructor
   */
  SimpleRecognitionTester();

  /*!
   * \brief Monitors the keyboard
   */
  void loop();

private:
  void newDataCallback(const rail_manipulation_msgs::SegmentedObjectList::ConstPtr &newData);

  void showObject(unsigned int index);

  ros::NodeHandle n, pnh;

  ros::Subscriber newDataSubscriber;
  ros::Publisher currentObjectPublisher;

  ros::ServiceClient recognizeClient;

  rail_manipulation_msgs::SegmentedObjectList objects;
  unsigned int index;

  boost::mutex objectMutex;
};

/*!
 * \brief A function to close ROS and exit the program.
 *
 * \param sig The signal value.
 */
void shutdown(int sig);

#endif
