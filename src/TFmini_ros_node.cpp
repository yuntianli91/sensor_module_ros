/*
 * @Description: 
 * @Author: Yuntian Li
 * @Github: https://github.com/yuntinali91
 * @Date: 2019-11-05 20:00:08
 * @LastEditors: Yuntian Li
 * @LastEditTime: 2019-11-06 14:56:40
 */
#include "TFmini.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tfmini_ros_node");
  ros::NodeHandle nh("~");
  std::string id = "range";
  std::string portName;
  int baud_rate;
  benewake::TFmini *tfmini_obj;

  nh.param("serial_port", portName, std::string("/dev/ttyUSB0"));
  nh.param("baud_rate", baud_rate, 115200);

  tfmini_obj = new benewake::TFmini(portName, baud_rate);
  ros::Publisher pub_range = nh.advertise<sensor_msgs::Range>(id, 1000, true);
  sensor_msgs::Range TFmini_range;
  TFmini_range.radiation_type = sensor_msgs::Range::INFRARED;
  TFmini_range.field_of_view = 0.04;
  TFmini_range.min_range = 0.3;
  TFmini_range.max_range = 12;
  TFmini_range.header.frame_id = id;
  float dist = 0;
  ROS_INFO_STREAM("Start processing ...");

  int pub_rate;
  nh.getParam("/tfmini_ros_node/pub_rate", pub_rate);
  ros::Rate loop_rate(pub_rate);

  while(ros::master::check() && ros::ok())
  {
    ros::spinOnce();
    dist = tfmini_obj->getDist();
    if(dist > 0 && dist < TFmini_range.max_range)
    {
      TFmini_range.range = dist;
      TFmini_range.header.stamp = ros::Time::now();
      pub_range.publish(TFmini_range);
    }
    else if(dist == -1.0)
    {
      ROS_ERROR_STREAM("Failed to read data. TFmini ros node stopped!");
      break;
    }
    else if(dist == 0.0)
    {
      ROS_ERROR_STREAM("Data validation error!");
    }
    loop_rate.sleep();
  }

  tfmini_obj->closePort();
}
