#pragma once

#include <condition_variable>
#include <functional>
#include <memory>
#include <optional>
#include <iostream>
#include <thread>

namespace apriltag_marker_detector {

template <typename... Args> class AsyncExecutor {

  public:
	using Handler = std::function<void(Args...)>;

	AsyncExecutor(Handler handler)
	    : handler(handler),
	      thread(std::bind(&AsyncExecutor::run_executor, this)) {}

	~AsyncExecutor() {
		{
			std::lock_guard<std::mutex> lock(mutex);
			running = false;
			cv.notify_all();
		}
		thread.join();
	}

	void feed(Args... args) {
		std::lock_guard<std::mutex> lock(mutex);
		task = std::make_tuple(args...);
		cv.notify_one();
	}

  private:
	Handler handler;
	bool running = true;
	std::optional<std::tuple<Args...>> task;
	std::mutex mutex;
	std::condition_variable cv;
	std::thread thread;

	void run_executor() {
		for (;;) {
			std::unique_lock<std::mutex> lock(mutex);
			cv.wait(lock, [&] { return !running || task.has_value(); });
			if (!running) {
				return;
			}
			std::tuple<Args...> task_args = *task;
			task = std::nullopt;
			lock.unlock();

			std::apply(handler, task_args);
		}
	}
};

}; // namespace apriltag_marker_detector
