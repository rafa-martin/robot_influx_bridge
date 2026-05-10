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
#ifndef ROBOT_INFLUX_BRIDGE__PERSISTENT_BATCH_STORE_HPP_
#define ROBOT_INFLUX_BRIDGE__PERSISTENT_BATCH_STORE_HPP_

#include <rclcpp/logger.hpp>

#include <deque>
#include <filesystem>
#include <istream>
#include <memory>
#include <optional>
#include <ostream>
#include <string>
#include <vector>

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

} // namespace robot_influx_bridge

#endif // ROBOT_INFLUX_BRIDGE__PERSISTENT_BATCH_STORE_HPP_
