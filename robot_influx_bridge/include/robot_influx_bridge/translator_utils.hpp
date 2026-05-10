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
#ifndef ROBOT_INFLUX_BRIDGE__TRANSLATOR_UTILS_HPP_
#define ROBOT_INFLUX_BRIDGE__TRANSLATOR_UTILS_HPP_

#include <builtin_interfaces/msg/time.hpp>
#include <robot_influx_bridge/line_protocol_utils.hpp>
#include <robot_influx_bridge/translator_base.hpp>

#include <sstream>
#include <string>
#include <vector>

namespace robot_influx_bridge {

std::string build_series_name(const TranslatorContext& context, const std::string& default_measurement);

void append_timestamp_if_valid(std::ostringstream& stream, const builtin_interfaces::msg::Time& stamp);

} // namespace robot_influx_bridge

#endif // ROBOT_INFLUX_BRIDGE__TRANSLATOR_UTILS_HPP_
