#include <robot_influx_bridge/influx_error.hpp>

namespace robot_influx_bridge {

InfluxError::InfluxError(std::string msg, long http, CURLcode curl, std::string body)
: std::runtime_error(std::move(msg)), http_status_(http), curl_code_(curl), body_(std::move(body))
{}

}  // namespace robot_influx_bridge
