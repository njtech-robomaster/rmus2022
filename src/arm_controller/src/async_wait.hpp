#pragma once

#include <ros/ros.h>

class AsyncWaiter {
  public:
	void wait(ros::Duration duration, std::function<void()> cb);

  private:
	ros::NodeHandle nh;
	std::optional<ros::Timer> timer;
};
