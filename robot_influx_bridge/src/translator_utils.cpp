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

#include <robot_influx_bridge/translator_utils.hpp>

#include <algorithm>
#include <utility>
#include <vector>

namespace robot_influx_bridge {

std::string build_series_name(const TranslatorContext& context, const std::string& default_measurement) {
    const std::string measurement = line_protocol::resolve_measurement_name(context.measurement, default_measurement);

    std::ostringstream series;
    series << line_protocol::escape_measurement(measurement);

    std::vector<std::pair<std::string, std::string>> tags{context.static_tags.begin(), context.static_tags.end()};
    std::sort(tags.begin(), tags.end(), [](const auto& lhs, const auto& rhs) { return lhs.first < rhs.first; });

    for (const auto& [key, value] : tags) {
        series << ',' << line_protocol::escape_field_key(key) << '=' << line_protocol::escape_tag_value(value);
    }

    return series.str();
}

void append_timestamp_if_valid(std::ostringstream& stream, const builtin_interfaces::msg::Time& stamp) {
    const int64_t time_ns = line_protocol::stamp_to_nanoseconds(stamp);
    if (time_ns != 0) {
        // Append timestamp in nanoseconds if valid (non-zero)
        stream << ' ' << time_ns;
    }
    else {
        // No timestamp available, Use current time when the point is written to InfluxDB
        auto const now = std::chrono::system_clock::now();
        stream << ' ' << std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
    }
}

} // namespace robot_influx_bridge
