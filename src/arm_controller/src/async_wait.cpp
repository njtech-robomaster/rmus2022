#include "async_wait.hpp"

void AsyncWaiter::wait(ros::Duration duration, std::function<void()> cb) {
	if (this->timer != std::nullopt) {
		throw std::runtime_error("timer pending");
	}
	this->timer = nh.createTimer(
	    duration,
	    [this, cb](auto) {
		    this->timer = std::nullopt;
		    cb();
	    },
	    true, true);
}
