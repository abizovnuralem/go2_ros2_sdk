#pragma once

#include <atomic>
#include <condition_variable>
#include <deque>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

#include "websocket_logging.hpp"

namespace foxglove {

class CallbackQueue {
public:
  CallbackQueue(LogCallback logCallback, size_t numThreads = 1)
      : _logCallback(logCallback)
      , _quit(false) {
    for (size_t i = 0; i < numThreads; ++i) {
      _workerThreads.push_back(std::thread(&CallbackQueue::doWork, this));
    }
  }

  ~CallbackQueue() {
    stop();
  }

  void stop() {
    _quit = true;
    _cv.notify_all();
    for (auto& thread : _workerThreads) {
      thread.join();
    }
  }

  void addCallback(std::function<void(void)> cb) {
    if (_quit) {
      return;
    }
    std::unique_lock<std::mutex> lock(_mutex);
    _callbackQueue.push_back(cb);
    _cv.notify_one();
  }

private:
  void doWork() {
    while (!_quit) {
      std::unique_lock<std::mutex> lock(_mutex);
      _cv.wait(lock, [this] {
        return (_quit || !_callbackQueue.empty());
      });
      if (_quit) {
        break;
      } else if (!_callbackQueue.empty()) {
        std::function<void(void)> cb = _callbackQueue.front();
        _callbackQueue.pop_front();
        lock.unlock();
        try {
          cb();
        } catch (const std::exception& ex) {
          // Should never get here if we catch all exceptions in the callbacks.
          const std::string msg =
            std::string("Caught unhandled exception in calback_queue") + ex.what();
          _logCallback(WebSocketLogLevel::Error, msg.c_str());
        } catch (...) {
          _logCallback(WebSocketLogLevel::Error, "Caught unhandled exception in calback_queue");
        }
      }
    }
  }

  LogCallback _logCallback;
  std::atomic<bool> _quit;
  std::mutex _mutex;
  std::condition_variable _cv;
  std::deque<std::function<void(void)>> _callbackQueue;
  std::vector<std::thread> _workerThreads;
};

}  // namespace foxglove
