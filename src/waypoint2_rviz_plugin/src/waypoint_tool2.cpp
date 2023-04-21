/*
 * @Author: jia
 * @Date: 2023-03-16 22:06:38
 * @LastEditors: jia
 * @LastEditTime: 2023-04-19 16:04:28
 * @Description: 请填写简
 */
#include "waypoint_tool2.h"

namespace rviz
{
WaypointTool2::WaypointTool2()
{
  shortcut_key_ = 'w';

  topic_property_ = new StringProperty("Topic", "waypoint", "The topic on which to publish navigation waypionts.",
                                       getPropertyContainer(), SLOT(updateTopic()), this);
}

void WaypointTool2::onInitialize()
{
  PoseTool::onInitialize();
  setName("Waypoint2");
  updateTopic();
  vehicle_z = 0;
}

void WaypointTool2::updateTopic()
{
  sub_ = nh_.subscribe<nav_msgs::Odometry> ("state_estimation", 5, &WaypointTool2::odomHandler, this);
  pub_ = nh_.advertise<geometry_msgs::PointStamped>("/robot2/way_point2", 5);
  pub_joy_ = nh_.advertise<sensor_msgs::Joy>("joy", 5);
}

void WaypointTool2::odomHandler(const nav_msgs::Odometry::ConstPtr& odom)
{
  vehicle_z = odom->pose.pose.position.z;
}

void WaypointTool2::onPoseSet(double x, double y, double theta)
{
  sensor_msgs::Joy joy;

  joy.axes.push_back(0);
  joy.axes.push_back(0);
  joy.axes.push_back(-1.0);
  joy.axes.push_back(0);
  joy.axes.push_back(1.0);
  joy.axes.push_back(1.0);
  joy.axes.push_back(0);
  joy.axes.push_back(0);

  joy.buttons.push_back(0);
  joy.buttons.push_back(0);
  joy.buttons.push_back(0);
  joy.buttons.push_back(0);
  joy.buttons.push_back(0);
  joy.buttons.push_back(0);
  joy.buttons.push_back(0);
  joy.buttons.push_back(1);
  joy.buttons.push_back(0);
  joy.buttons.push_back(0);
  joy.buttons.push_back(0);

  joy.header.stamp = ros::Time::now();
  joy.header.frame_id = "waypoint_tool";
  pub_joy_.publish(joy);

  geometry_msgs::PointStamped waypoint;
  waypoint.header.frame_id = "map2";
  waypoint.header.stamp = joy.header.stamp;
  waypoint.point.x = x;
  waypoint.point.y = y;
  waypoint.point.z = vehicle_z;

  pub_.publish(waypoint);
  usleep(10000);
  pub_.publish(waypoint);
}
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz::WaypointTool2, rviz::Tool)
