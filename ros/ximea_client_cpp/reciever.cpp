#include "reciever.h"

void receiver::imageCallback (const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
    {
      cv_ptr = cv_bridge::toCvCopy (msg, sensor_msgs::image_encodings::BGR8);
    }
  catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what ());
      return;
    }
}


void receiver::getImage (int index)
{

}


void receiver::getImages ()
{
  ximea_msgs::Capture srv;
  if (this->client.call(srv))
  {
    ROS_INFO("Got images");
  }
  else
  {
    ROS_ERROR("Error calling ximea_msgs::Capture service");
  }
}


receiver::receiver (ros::NodeHandle *nh)
{
  this->nh = nh;
  this->it = image_transport::ImageTransport
  it (*nh);
  this->client = nh.serviceClient<ximea_msgs::Capture>("capture");
}

receiver::Ptr receiver::create (ros::NodeHandle *nh)
{
  return receiver::Ptr (new receiver (nh));
}
