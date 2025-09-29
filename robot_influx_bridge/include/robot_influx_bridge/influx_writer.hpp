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

#include <rclcpp/logger.hpp>

namespace robot_influx_bridge {

class InfluxWriter {
public:
  InfluxWriter(const rclcpp::Logger& logger,
               const std::string& url, const std::string& org,
               const std::string& bucket, const std::string& token,
               size_t max_batch=1000, size_t max_queue=100000,
               std::chrono::milliseconds flush=std::chrono::milliseconds(500),
               bool use_gzip=false);
  ~InfluxWriter();
  bool enqueue(std::string line);
  std::string last_error() const;
private:
  void run();
  bool post(const std::string& body);
  void set_last_error(std::string message) const;
  std::string endpoint_, token_;
  size_t max_batch_, max_queue_;
  std::chrono::milliseconds flush_;
  rclcpp::Logger logger_;
  std::mutex m_;
  std::condition_variable cv_;
  std::queue<std::string> q_;
  std::atomic<bool> stop_{false};
  std::thread th_;
  mutable std::mutex error_mutex_;
  mutable std::string last_error_;
  std::atomic<bool> has_logged_success_{false};
  std::atomic<bool> error_since_last_success_{false};
  bool use_gzip_;
};

} // namespace robot_influx_bridge
