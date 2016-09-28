/*
 * receiver
 */

#pragma once
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <string>


class receiver {
  typedef typename boost::shared_ptr<receiver> Ptr;
protected:
  receiver(ros::NodeHandle *nh);
  ~receiver();
  ros::NodeHandle *nh;
  image_transport::ImageTransport it;
  image_transport::Subscriber sub;
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);

public:
  Ptr create(ros::NodeHandle *nh);
  void getImage(int index);
  void getImages();

private:
  ros::ServiceClient client;
};
