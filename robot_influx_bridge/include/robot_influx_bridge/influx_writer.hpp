// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Rafael Martin

#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <optional>
#include <queue>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/logger.hpp>

#include <robot_influx_bridge/persistent_batch_store.hpp>

namespace robot_influx_bridge {

/// Background worker that batches line protocol messages and uploads them to InfluxDB.
///
/// The writer exposes a single-threaded enqueue API. Messages are buffered until either
/// `max_batch` lines are accumulated or `flush` elapses, whichever happens first. The
/// `last_error()` accessor can be polled to surface the latest HTTP or network issue.
class InfluxWriter {
public:
  InfluxWriter(const rclcpp::Logger& logger,
               const std::string& url, const std::string& org,
               const std::string& bucket, const std::string& token,
               size_t max_batch = 1000, size_t max_queue = 100000,
               std::chrono::milliseconds flush = std::chrono::milliseconds(500),
               bool use_gzip = false,
               const std::string& persistence_path = std::string(),
               size_t persistence_max_megabytes = 64);
  ~InfluxWriter();

  /// Queue a line protocol string for upload. Returns `false` if the queue is full.
  bool enqueue(std::string line);

  /// Retrieve the last error message reported by the uploader. Returns an empty string
  /// when no error has been observed.
  std::string last_error() const;

private:
  void run();
  struct SendAttemptResult {
    bool success{false};
    bool retryable{false};
    std::string reason;
  };
  SendAttemptResult send_batch(const std::vector<std::string>& lines);
  std::string build_body(const std::vector<std::string>& lines) const;
  std::chrono::milliseconds backoff_duration(size_t attempt) const;
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
  bool logged_shutdown_flush_started_{false};
  bool logged_shutdown_flush_completed_{false};
  std::unique_ptr<PersistentBatchStore> persistent_store_;
  std::optional<typename PersistentBatchStore::PendingBatch> pending_disk_batch_;
  size_t pending_disk_attempts_{0};
  bool logged_connection_loss_{false};
};

}  // namespace robot_influx_bridge
