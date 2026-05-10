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


#include <robot_influx_bridge/persistent_batch_store.hpp>

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iterator>
#include <utility>

#include <rclcpp/logging.hpp>

namespace robot_influx_bridge {

namespace {
constexpr double kBytesPerMegabyte = 1024.0 * 1024.0;

double to_megabytes(uintmax_t bytes) {
  return static_cast<double>(bytes) / kBytesPerMegabyte;
}
}

bool TextBatchFormatter::serialize(const std::vector<std::string>& lines, std::ostream& os) const {
  for (const auto& line : lines) {
    os << line << '\n';
    if (!os) {
      return false;
    }
  }
  return true;
}

std::vector<std::string> TextBatchFormatter::deserialize(std::istream& is) const {
  std::vector<std::string> lines;
  std::string line;
  while (std::getline(is, line)) {
    lines.push_back(line);
  }
  if (!is.eof() && is.fail()) {
    return {};
  }
  return lines;
}

PersistentBatchStore::PersistentBatchStore(std::filesystem::path directory,
                                           uintmax_t max_bytes,
                                           rclcpp::Logger logger,
                                           std::shared_ptr<BatchFormatter> formatter)
: directory_(std::move(directory)),
  max_bytes_(max_bytes),
  logger_(logger),
  formatter_(std::move(formatter))
{}

void PersistentBatchStore::initialize() {
  if (initialized_) {
    return;
  }
  initialized_ = true;

  if (directory_.empty() || max_bytes_ == 0) {
    disabled_ = true;
    return;
  }

  std::error_code ec;
  std::filesystem::create_directories(directory_, ec);
  if (ec) {
    RCLCPP_ERROR(logger_, "Failed to create persistence directory '%s': %s", directory_.c_str(), ec.message().c_str());
    disabled_ = true;
    return;
  }

  load_existing();

  if (!queue_.empty()) {
    const auto pending = queue_.size();
    const char* plural = pending == 1 ? "" : "es";
    RCLCPP_WARN(logger_,
      "Recovered %zu persisted batch%s from previous run (%.2f/%.2f MB).",
      pending, plural, to_megabytes(current_bytes_), to_megabytes(max_bytes_));
  }
}

bool PersistentBatchStore::enabled() const noexcept {
  return !disabled_ && max_bytes_ > 0;
}

bool PersistentBatchStore::empty() const noexcept {
  return queue_.empty();
}

size_t PersistentBatchStore::pending_batches() const noexcept {
  return queue_.size();
}

uintmax_t PersistentBatchStore::current_bytes() const noexcept {
  return current_bytes_;
}

uintmax_t PersistentBatchStore::max_bytes() const noexcept {
  return max_bytes_;
}

const std::filesystem::path& PersistentBatchStore::directory() const noexcept {
  return directory_;
}

bool PersistentBatchStore::store(const std::vector<std::string>& lines) {
  if (!enabled()) {
    return false;
  }

  if (lines.empty()) {
    return true;
  }

  const std::string filename = make_filename();
  const auto path = directory_ / filename;

  std::ofstream file(path, std::ios::binary | std::ios::trunc);
  if (!file) {
    RCLCPP_ERROR(logger_, "Failed to open '%s' for writing persisted batch", path.c_str());
    return false;
  }

  if (!formatter_->serialize(lines, file)) {
    RCLCPP_ERROR(logger_, "Failed to persist batch to '%s'", path.c_str());
    file.close();
    std::error_code remove_ec;
    std::filesystem::remove(path, remove_ec);
    return false;
  }

  file.flush();
  if (!file) {
    RCLCPP_ERROR(logger_, "Failed to flush persisted batch '%s'", path.c_str());
    file.close();
    std::error_code remove_ec;
    std::filesystem::remove(path, remove_ec);
    return false;
  }
  file.close();

  std::error_code size_ec;
  const uintmax_t size = std::filesystem::file_size(path, size_ec);
  if (size_ec) {
    RCLCPP_ERROR(logger_, "Failed to determine size of persisted batch '%s': %s", path.c_str(), size_ec.message().c_str());
    std::error_code remove_ec;
    std::filesystem::remove(path, remove_ec);
    return false;
  }

  queue_.push_back(path);
  current_bytes_ += size;

  if (current_bytes_ > max_bytes_) {
    enforce_size_limit();
    if (current_bytes_ > max_bytes_) {
      RCLCPP_WARN(logger_,
        "Persisted batch '%s' (%.2f MB) exceeds disk budget of %.2f MB. Dropping latest batch.",
        path.filename().c_str(), to_megabytes(size), to_megabytes(max_bytes_));
      queue_.pop_back();
      current_bytes_ -= size;
      std::error_code remove_ec;
      std::filesystem::remove(path, remove_ec);
      return false;
    }
  }

  const double used = to_megabytes(current_bytes_);
  const double limit = to_megabytes(max_bytes_);
  RCLCPP_DEBUG(logger_, "Persisted batch '%s'. Disk queue usage: %.2f/%.2f MB", path.filename().c_str(), used, limit);

  return true;
}

std::optional<PersistentBatchStore::PendingBatch> PersistentBatchStore::next() {
  if (!enabled()) {
    return std::nullopt;
  }

  if (queue_.empty()) {
    return std::nullopt;
  }

  const auto& path = queue_.front();
  std::ifstream file(path, std::ios::binary);
  if (!file) {
    RCLCPP_ERROR(logger_, "Failed to open persisted batch '%s' for reading. Discarding.", path.c_str());
    drop_oldest(path);
    return std::nullopt;
  }

  auto lines = formatter_->deserialize(file);
  if (!file.eof() && file.fail()) {
    RCLCPP_ERROR(logger_, "Failed to read persisted batch '%s'. Discarding.", path.c_str());
    drop_oldest(path);
    return std::nullopt;
  }

  return PendingBatch{path, std::move(lines)};
}

void PersistentBatchStore::mark_processed(const PendingBatch& batch) {
  if (!enabled()) {
    return;
  }

  if (queue_.empty()) {
    return;
  }

  const auto& front = queue_.front();
  if (front != batch.path) {
    return;
  }

  std::error_code size_ec;
  const uintmax_t size = std::filesystem::file_size(front, size_ec);
  if (!size_ec && current_bytes_ >= size) {
    current_bytes_ -= size;
  }

  std::error_code remove_ec;
  std::filesystem::remove(front, remove_ec);
  if (remove_ec) {
    RCLCPP_WARN(logger_, "Failed to remove persisted batch '%s': %s", front.c_str(), remove_ec.message().c_str());
  }

  queue_.pop_front();
}

void PersistentBatchStore::load_existing() {
  queue_.clear();
  current_bytes_ = 0;

  std::vector<std::filesystem::directory_entry> entries;
  std::error_code ec;
  for (const auto& entry : std::filesystem::directory_iterator(directory_, ec)) {
    if (entry.is_regular_file()) {
      entries.push_back(entry);
    }
  }

  if (ec) {
    RCLCPP_ERROR(logger_, "Failed to scan persistence directory '%s': %s", directory_.c_str(), ec.message().c_str());
    disabled_ = true;
    queue_.clear();
    current_bytes_ = 0;
    return;
  }

  std::sort(entries.begin(), entries.end(),
    [](const auto& lhs, const auto& rhs) {
      return lhs.path().filename().string() < rhs.path().filename().string();
    });

  for (const auto& entry : entries) {
    std::error_code size_ec;
    const auto size = entry.file_size(size_ec);
    if (size_ec) {
      RCLCPP_WARN(logger_, "Failed to read size for '%s': %s", entry.path().c_str(), size_ec.message().c_str());
      continue;
    }
    queue_.push_back(entry.path());
    current_bytes_ += size;
  }
}

void PersistentBatchStore::enforce_size_limit() {
  if (max_bytes_ == 0) {
    return;
  }

  while (current_bytes_ > max_bytes_ && !queue_.empty()) {
    if (queue_.size() == 1) {
      break;
    }
    const auto path = queue_.front();
    drop_oldest(path);
  }
}

void PersistentBatchStore::drop_oldest(const std::filesystem::path& path) {
  std::error_code size_ec;
  const uintmax_t size = std::filesystem::file_size(path, size_ec);
  if (!size_ec && current_bytes_ >= size) {
    current_bytes_ -= size;
  }

  const double used = to_megabytes(current_bytes_);
  const double limit = to_megabytes(max_bytes_);
  RCLCPP_WARN(logger_, "Discarding persisted batch '%s' to maintain disk budget (%.2f/%.2f MB)",
    path.filename().c_str(), used, limit);

  std::error_code remove_ec;
  std::filesystem::remove(path, remove_ec);
  if (remove_ec) {
    RCLCPP_WARN(logger_, "Failed to remove persisted batch '%s': %s", path.c_str(), remove_ec.message().c_str());
  }

  if (!queue_.empty() && queue_.front() == path) {
    queue_.pop_front();
  } else {
    auto it = std::find(queue_.begin(), queue_.end(), path);
    if (it != queue_.end()) {
      queue_.erase(it);
    }
  }
}

std::string PersistentBatchStore::make_filename() {
  const auto now = std::chrono::system_clock::now();
  const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
  return "batch_" + std::to_string(ms) + "_" + std::to_string(sequence_++) + ".txt";
}

}  // namespace robot_influx_bridge
