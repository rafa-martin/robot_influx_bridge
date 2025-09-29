// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Rafael Martin

#pragma once

#include <stdexcept>
#include <string>

#include <curl/curl.h>

namespace robot_influx_bridge {

struct InfluxError : public std::runtime_error {
  InfluxError(std::string msg, long http=0, CURLcode curl=CURLcode(0), std::string body="");

  inline long http_status() const noexcept { return http_status_; }
  inline CURLcode curl_code() const noexcept { return curl_code_; }
  inline const std::string& body() const noexcept { return body_; }

private:
  long http_status_;
  CURLcode curl_code_;
  std::string body_;
};

struct InfluxNetworkError : public InfluxError {
  using InfluxError::InfluxError;
};

struct InfluxHttpError : public InfluxError {
  using InfluxError::InfluxError;
};

}  // namespace robot_influx_bridge
