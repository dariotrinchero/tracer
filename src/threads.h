#ifndef THREADS_H
#define THREADS_H

#include <vector>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <functional>
#include <future>
#include <atomic>

class ThreadPool {
  public:
	ThreadPool(size_t thread_count) : pool_stopping(false) {
		for (size_t i = 0; i < thread_count; i++) {
			workers.emplace_back([this] {
				while (true) {
					std::function<void()> task;
					{
						// lock queue to safely pop task
						std::unique_lock<std::mutex> lock(queue_mutex);

						// wait until there is task to process (or pool is stopping)
						condition.wait(lock, [this] { return pool_stopping || !tasks.empty(); });
						if (pool_stopping && tasks.empty()) return;

						// retrieve task
						task = std::move(tasks.front());
						tasks.pop();
					} // unlock queue

					task();
				}
			});
		}
	}

	ThreadPool() : ThreadPool(std::thread::hardware_concurrency()) {}

	inline int num_threads() const { return workers.size(); }

	template <typename F, typename... Args>
	auto enqueue(F&& f, Args&&... args) -> std::future<typename std::result_of<F(Args...)>::type> {
		using return_type = typename std::result_of<F(Args...)>::type;

		// create packaged task (pre-bind arguments)
		auto task = std::make_shared<std::packaged_task<return_type()>>(
			std::bind(std::forward<F>(f), std::forward<Args>(args)...)
		);

		{ // lock queue & push new task
			std::unique_lock<std::mutex> lock(queue_mutex);
			if (pool_stopping) {
				throw std::runtime_error("ThreadPool is stopping, cannot enqueue new tasks.");
			}
			tasks.emplace([task]() { (*task)(); });
		}
		condition.notify_one(); // notify worker thread of new task

		// return future for task's return value
		return task->get_future();
	}

	~ThreadPool() {
		{ // lock queue to set pool_stopping flag
			std::unique_lock<std::mutex> lock(queue_mutex);
			pool_stopping = true;
		}
		condition.notify_all(); // notify all workers of pool stopping

		// wait for threads to exit before destroying
		for (std::thread& worker : workers) worker.join();
	}

  private:
	std::vector<std::thread>          workers; // list of worker threads
	std::queue<std::function<void()>> tasks; // queue of tasks to execute
	std::mutex                        queue_mutex;
	std::atomic<bool>                 pool_stopping; // flag indicating pool is stopping
	std::condition_variable           condition; // notifies workers of changes to queue or stop flag
};

#endif
