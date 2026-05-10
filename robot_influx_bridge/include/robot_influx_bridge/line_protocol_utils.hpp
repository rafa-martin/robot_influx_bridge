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
#ifndef ROBOT_INFLUX_BRIDGE__LINE_PROTOCOL_UTILS_HPP_
#define ROBOT_INFLUX_BRIDGE__LINE_PROTOCOL_UTILS_HPP_

#include <builtin_interfaces/msg/time.hpp>

#include <cstdint>
#include <string>

namespace robot_influx_bridge::line_protocol {

inline std::string escape_measurement(const std::string& value) {
    std::string escaped;
    escaped.reserve(value.size());
    for (char character : value) {
        if (character == ' ' || character == ',') {
            escaped.push_back('\\');
        }
        escaped.push_back(character);
    }
    return escaped;
}

inline std::string escape_tag_value(const std::string& value) {
    std::string escaped;
    escaped.reserve(value.size());
    for (char character : value) {
        if (character == ' ' || character == ',' || character == '=') {
            escaped.push_back('\\');
        }
        escaped.push_back(character);
    }
    return escaped;
}

inline std::string escape_field_key(const std::string& value) {
    return escape_tag_value(value);
}

inline std::string escape_string_field_value(const std::string& value) {
    std::string escaped = "\"";
    escaped.reserve(value.size() + 2U);
    for (char character : value) {
        if (character == '"') {
            escaped.push_back('\\');
        }
        escaped.push_back(character);
    }
    escaped.push_back('"');
    return escaped;
}

inline int64_t stamp_to_nanoseconds(const builtin_interfaces::msg::Time& stamp) {
    return static_cast<int64_t>(stamp.sec) * 1000000000LL + static_cast<int64_t>(stamp.nanosec);
}

inline std::string resolve_measurement_name(const std::string& configured, const std::string& fallback) {
    return configured.empty() ? fallback : configured;
}

} // namespace robot_influx_bridge::line_protocol

#endif // ROBOT_INFLUX_BRIDGE__LINE_PROTOCOL_UTILS_HPP_
