// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Rafael Martin

#pragma once

#include <deque>
#include <filesystem>
#include <istream>
#include <memory>
#include <optional>
#include <ostream>
#include <string>
#include <vector>

#include <rclcpp/logger.hpp>

namespace robot_influx_bridge {

class BatchFormatter {
public:
  virtual ~BatchFormatter() = default;
  virtual bool serialize(const std::vector<std::string>& lines, std::ostream& os) const = 0;
  virtual std::vector<std::string> deserialize(std::istream& is) const = 0;
};

class TextBatchFormatter : public BatchFormatter {
public:
  bool serialize(const std::vector<std::string>& lines, std::ostream& os) const override;
  std::vector<std::string> deserialize(std::istream& is) const override;
};

class PersistentBatchStore {
public:
  struct PendingBatch {
    std::filesystem::path path;
    std::vector<std::string> lines;
  };

  PersistentBatchStore(std::filesystem::path directory,
                       uintmax_t max_bytes,
                       rclcpp::Logger logger,
                       std::shared_ptr<BatchFormatter> formatter = std::make_shared<TextBatchFormatter>());

  void initialize();
  bool enabled() const noexcept;
  bool empty() const noexcept;
  size_t pending_batches() const noexcept;
  uintmax_t current_bytes() const noexcept;
  uintmax_t max_bytes() const noexcept;
  const std::filesystem::path& directory() const noexcept;

  bool store(const std::vector<std::string>& lines);
  std::optional<PendingBatch> next();
  void mark_processed(const PendingBatch& batch);

private:
  void load_existing();
  void enforce_size_limit();
  void drop_oldest(const std::filesystem::path& path);
  std::string make_filename();

  std::filesystem::path directory_;
  uintmax_t max_bytes_;
  rclcpp::Logger logger_;
  std::shared_ptr<BatchFormatter> formatter_;
  std::deque<std::filesystem::path> queue_;
  uintmax_t current_bytes_{0};
  bool initialized_{false};
  uint64_t sequence_{0};
  bool disabled_{false};
};

}  // namespace robot_influx_bridge
