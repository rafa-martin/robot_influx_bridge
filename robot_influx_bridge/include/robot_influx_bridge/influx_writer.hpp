// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Rafael Martin

#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <vector>

namespace robot_influx_bridge {

class InfluxWriter {
public:
  InfluxWriter(const std::string& url, const std::string& org,
               const std::string& bucket, const std::string& token,
               size_t max_batch=1000, size_t max_queue=100000,
               std::chrono::milliseconds flush=std::chrono::milliseconds(500));
  ~InfluxWriter();
  bool enqueue(std::string line);
private:
  void run();
  bool post(const std::string& body);
  std::string endpoint_, token_;
  size_t max_batch_, max_queue_;
  std::chrono::milliseconds flush_;
  std::mutex m_;
  std::condition_variable cv_;
  std::queue<std::string> q_;
  std::atomic<bool> stop_{false};
  std::thread th_;
};

} // namespace robot_influx_bridge
