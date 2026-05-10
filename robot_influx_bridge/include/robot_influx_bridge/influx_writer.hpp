// Copyright 2025, Rafael Martin. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of Rafael Martin nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#pragma once
#ifndef ROBOT_INFLUX_BRIDGE__INFLUX_WRITER_HPP_
#define ROBOT_INFLUX_BRIDGE__INFLUX_WRITER_HPP_

#include <rclcpp/logger.hpp>
#include <robot_influx_bridge/persistent_batch_store.hpp>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <queue>
#include <string>
#include <thread>
#include <vector>

namespace robot_influx_bridge {

/// Background worker that batches line protocol messages and uploads them to InfluxDB.
///
/// The writer exposes a single-threaded enqueue API. Messages are buffered until either
/// `max_batch` lines are accumulated or `flush` elapses, whichever happens first. The
/// `last_error()` accessor can be polled to surface the latest HTTP or network issue.
class InfluxWriter {
public:
    InfluxWriter(const rclcpp::Logger& logger,
                 const std::string& url,
                 const std::string& org,
                 const std::string& bucket,
                 const std::string& token,
                 size_t max_batch = 1000,
                 size_t max_queue = 100000,
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

    /// Current number of queued line protocol messages waiting to be batched.
    size_t queue_depth() const noexcept;

    /// Maximum configured queue capacity.
    size_t max_queue_size() const noexcept;

    /// Total number of persisted batches waiting on disk.
    size_t pending_batch_count() const noexcept;

    /// Bytes currently consumed by the persistence directory.
    uintmax_t persistence_bytes_used() const noexcept;

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
    void update_persistence_metrics();

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
    std::atomic<size_t> queue_depth_{0};
    std::atomic<size_t> pending_persisted_batches_{0};
    std::atomic<uintmax_t> persistence_bytes_used_{0};
};

} // namespace robot_influx_bridge

#endif // ROBOT_INFLUX_BRIDGE__INFLUX_WRITER_HPP_
