#include <curl/curl.h>

#include <algorithm>
#include <chrono>
#include <cctype>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <vector>

#include <zlib.h>

#include <robot_influx_bridge/influx_error.hpp>
#include <robot_influx_bridge/influx_writer.hpp>

#include <rclcpp/logging.hpp>

// HELPER FUNCTIONS
namespace {
size_t write_cb(char* ptr, size_t size, size_t nmemb, void* userdata) {
  auto* out = static_cast<std::string*>(userdata);
  out->append(ptr, size * nmemb);
  return size * nmemb;
}

size_t header_cb(char* buffer, size_t size, size_t nitems, void* userdata) {
  const size_t total = size * nitems;
  auto* content_type = static_cast<std::string*>(userdata);
  std::string line(buffer, total);
  const std::string key = "Content-Type:";
  if (line.size() >= key.size() && std::equal(key.begin(), key.end(), line.begin(),
      [](char a, char b){ return std::tolower(a) == std::tolower(b); })) {
    // value after colon
    auto pos = line.find(':');
    if (pos != std::string::npos) {
      *content_type = line.substr(pos + 1);
      // trim
      auto& s = *content_type;
      s.erase(0, s.find_first_not_of(" \t\r\n"));
      s.erase(s.find_last_not_of(" \t\r\n") + 1);
    }
  }
  return total;
}

// very small JSON extractor for {"message":"...","code":"..."}
std::string extract_json_string(const std::string& json, const std::string& key) {
  const std::string q = "\"" + key + "\"";
  auto k = json.find(q);
  if (k == std::string::npos) return {};
  k = json.find(':', k);
  if (k == std::string::npos) return {};
  k = json.find('"', k);
  if (k == std::string::npos) return {};
  auto e = json.find('"', k + 1);
  if (e == std::string::npos) return {};
  return json.substr(k + 1, e - k - 1);
}

bool equals_ignore_case(const std::string& a, const std::string& b) {
  return a.size() == b.size() && std::equal(a.begin(), a.end(), b.begin(),
    [](char lhs, char rhs) {
      return std::tolower(static_cast<unsigned char>(lhs)) ==
             std::tolower(static_cast<unsigned char>(rhs));
    });
}

std::string trim(std::string s) {
  s.erase(0, s.find_first_not_of(" \t\r\n"));
  s.erase(s.find_last_not_of(" \t\r\n") + 1);
  return s;
}

const char* plural_suffix(size_t count, const char* suffix = "s") {
  return count == 1 ? "" : suffix;
}

std::string compress_gzip_string(const std::string& input) {
  if (input.empty()) {
    // zlib still produces a valid gzip stream for empty input, but avoid work
    // by returning the precomputed gzip header/footer for an empty payload.
    static const unsigned char empty_gzip[] = {
      0x1f, 0x8b, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03,
      0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    };
    return std::string(reinterpret_cast<const char*>(empty_gzip), sizeof(empty_gzip));
  }

  z_stream stream{};
  stream.next_in = reinterpret_cast<Bytef*>(const_cast<char*>(input.data()));
  stream.avail_in = static_cast<uInt>(input.size());

  const int window_bits = 15 + 16;  // 15 = max window, +16 to add gzip header
  const int result = deflateInit2(&stream, Z_DEFAULT_COMPRESSION, Z_DEFLATED,
                                  window_bits, 8, Z_DEFAULT_STRATEGY);
  if (result != Z_OK) {
    throw std::runtime_error("Failed to initialize zlib stream for gzip compression");
  }

  std::string output;
  std::vector<unsigned char> buffer(16384);
  int flush = Z_NO_FLUSH;

  do {
    stream.next_out = buffer.data();
    stream.avail_out = static_cast<uInt>(buffer.size());

    if (stream.avail_in == 0) {
      flush = Z_FINISH;
    }

    const int deflate_result = deflate(&stream, flush);
    if (deflate_result == Z_STREAM_ERROR) {
      deflateEnd(&stream);
      throw std::runtime_error("Error while compressing payload with gzip");
    }

    const size_t produced = buffer.size() - stream.avail_out;
    output.append(reinterpret_cast<const char*>(buffer.data()), produced);

    if (deflate_result == Z_STREAM_END) {
      break;
    }
  } while (stream.avail_out == 0 || flush != Z_FINISH);

  deflateEnd(&stream);
  return output;
}

const char* http_reason_phrase(long status) {
  switch (status) {
    case 400: return "Bad Request";
    case 401: return "Unauthorized";
    case 402: return "Payment Required";
    case 403: return "Forbidden";
    case 404: return "Not Found";
    case 405: return "Method Not Allowed";
    case 408: return "Request Timeout";
    case 409: return "Conflict";
    case 413: return "Payload Too Large";
    case 415: return "Unsupported Media Type";
    case 422: return "Unprocessable Entity";
    case 429: return "Too Many Requests";
    case 500: return "Internal Server Error";
    case 502: return "Bad Gateway";
    case 503: return "Service Unavailable";
    case 504: return "Gateway Timeout";
    default: return nullptr;
  }
}

std::string describe_error(const robot_influx_bridge::InfluxError& error) {
  const long http = error.http_status();
  const CURLcode curl_code = error.curl_code();
  const std::string trimmed_message = trim(std::string(error.what()));
  std::ostringstream oss;

  if (http > 0) {
    oss << "HTTP " << http;
    if (const char* reason = http_reason_phrase(http)) {
      oss << " " << reason;
    }

    if (!trimmed_message.empty()) {
      const std::string prefix = "HTTP " + std::to_string(http);
      std::string remainder = trimmed_message;
      if (remainder.size() >= prefix.size() &&
          std::equal(prefix.begin(), prefix.end(), remainder.begin(),
            [](char lhs, char rhs) {
              return std::tolower(static_cast<unsigned char>(lhs)) ==
                     std::tolower(static_cast<unsigned char>(rhs));
            })) {
        remainder = trim(remainder.substr(prefix.size()));
      }

      if (!remainder.empty()) {
        const char* reason = http_reason_phrase(http);
        if (!(reason && equals_ignore_case(remainder, reason))) {
          oss << ": " << remainder;
        }
      }
    }
    return oss.str();
  }

  if (curl_code != CURLE_OK) {
    const char* curl_message = curl_easy_strerror(curl_code);
    oss << "Network error";
    if (curl_message && curl_message[0] != '\0') {
      oss << " (" << curl_message << ")";
    }
    if (!trimmed_message.empty() && !(curl_message && equals_ignore_case(trimmed_message, curl_message))) {
      oss << ": " << trimmed_message;
    }
    return oss.str();
  }

  if (!trimmed_message.empty()) {
    return trimmed_message;
  }

  return "Unknown error";
}

std::string parse_error_payload(
    long http_status, const std::string& body, const std::string& content_type) {
  if (body.empty()) {
    return {};
  }

  std::string lowered = content_type;
  std::transform(lowered.begin(), lowered.end(), lowered.begin(),
    [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

  std::string parsed;
  if (lowered.find("application/json") != std::string::npos) {
    const auto code = extract_json_string(body, "code");
    const auto message = extract_json_string(body, "message");
    if (!message.empty()) {
      parsed = message;
    }
    if (!code.empty()) {
      parsed = code + (parsed.empty() ? std::string() : ": " + parsed);
    }
  }

  if (parsed.empty()) {
    parsed = trim(body);
  }

  if (parsed.empty()) {
    if (http_status > 0) {
      parsed = "HTTP " + std::to_string(http_status);
    } else {
      parsed = "HTTP error";
    }
  }

  return parsed;
}
}  // namespace

using namespace std::chrono_literals;
namespace robot_influx_bridge {

InfluxWriter::InfluxWriter(const rclcpp::Logger& logger,
    const std::string& url,const std::string& org,
    const std::string& bucket,const std::string& token,size_t max_batch,size_t max_queue,
    std::chrono::milliseconds flush,bool use_gzip)
: token_(token), max_batch_(max_batch), max_queue_(max_queue), flush_(flush), logger_(logger),
  use_gzip_(use_gzip)
{
  endpoint_ = url + "/api/v2/write?org=" + org + "&bucket=" + bucket + "&precision=ns";
  curl_global_init(CURL_GLOBAL_DEFAULT);
  th_ = std::thread(&InfluxWriter::run, this);
}
InfluxWriter::~InfluxWriter(){
  stop_ = true; cv_.notify_all();
  if (th_.joinable()) th_.join();
  curl_global_cleanup();
}

bool InfluxWriter::enqueue(std::string line){
  std::unique_lock<std::mutex> lk(m_);
  // debug
  // RCLCPP_INFO(logger_, "Enqueuing InfluxDB line: %s", line.c_str());
  if(q_.size() >= max_queue_) {
    lk.unlock();
    set_last_error("Queue is full");
    return false;
  }
  q_.push(std::move(line));
  cv_.notify_one();
  lk.unlock();
  set_last_error("");
  return true;
}

std::string InfluxWriter::last_error() const {
  std::lock_guard<std::mutex> lk(error_mutex_);
  return last_error_;
}

void InfluxWriter::run(){
  std::vector<std::string> batch;
  batch.reserve(max_batch_);
  while (true) {
    std::unique_lock<std::mutex> lk(m_);
    if (batch.empty()) {
      cv_.wait_for(lk, flush_, [&] {
        return stop_ || q_.size() >= max_batch_ || !q_.empty();
      });
      while (!q_.empty() && batch.size() < max_batch_) {
        batch.push_back(std::move(q_.front()));
        q_.pop();
      }
    }

    const bool shutting_down = stop_.load(std::memory_order_relaxed);
    const bool queue_empty = q_.empty();

    if (batch.empty()) {
      if (shutting_down && queue_empty) {
        if (logged_shutdown_flush_started_ && !logged_shutdown_flush_completed_) {
          logged_shutdown_flush_completed_ = true;
          RCLCPP_INFO(logger_, "Shutdown flush completed. All buffered measurements were uploaded.");
        }
        break;
      }
      continue;
    }

    lk.unlock();

    const size_t lines = batch.size();

    if (shutting_down && !logged_shutdown_flush_started_) {
      logged_shutdown_flush_started_ = true;
      const char* plural = plural_suffix(lines);
      RCLCPP_INFO(logger_,
        "Shutdown requested. Flushing %zu queued measurement%s before exit...",
        lines, plural);
    }

    std::string body;
    body.reserve(batch.size() * 64);
    for (size_t i = 0; i < batch.size(); ++i) {
      body += batch[i];
      body.push_back('\n');
    }

    const char* plural = plural_suffix(lines);
    bool success = false;
    int attempt = 0;
    while (true) {
      try {
        // debug
        RCLCPP_INFO(logger_, "Posting %zu measurement%s to InfluxDB...", lines, plural);
        if (post(body)) {
          success = true;
        }
      } catch (const InfluxError& e) {
        error_since_last_success_.store(true, std::memory_order_relaxed);
        const std::string reason = describe_error(e);
        RCLCPP_WARN(logger_, "InfluxDB write attempt %d failed: %s", attempt + 1, reason.c_str());
        set_last_error(reason);
      }

      if (success) {
        set_last_error("");
        const bool had_error = error_since_last_success_.exchange(false, std::memory_order_relaxed);
        if (!has_logged_success_.exchange(true, std::memory_order_relaxed)) {
          RCLCPP_INFO(logger_, "InfluxDB write succeeded (%zu line%s). Data upload is active.",
            lines, plural);
        } else if (had_error) {
          RCLCPP_INFO(logger_, "InfluxDB write succeeded (%zu line%s). Data upload has recovered.",
            lines, plural);
        } else {
          RCLCPP_DEBUG(logger_, "InfluxDB write succeeded (%zu line%s).", lines, plural);
        }
        break;
      }

      const bool exceeded_attempts = !shutting_down && attempt >= 4;
      if (exceeded_attempts) {
        RCLCPP_ERROR(logger_,
          "Abandoning batch after %d failed attempts; dropping %zu measurement%s.",
          attempt + 1, lines, plural);
        break;
      }

      if (shutting_down) {
        RCLCPP_WARN(logger_,
          "Shutdown flush attempt %d failed; will keep retrying until shutdown is forced.",
          attempt + 1);
      }

      const int capped_attempt = std::min(attempt, 6);
      const auto backoff = std::chrono::milliseconds(std::min(5000, 50 * (1 << capped_attempt)));
      std::this_thread::sleep_for(backoff);
      ++attempt;
    }

    if (!success) {
      if (!shutting_down) {
        batch.clear();
      }
      continue;
    }

    if (shutting_down && logged_shutdown_flush_started_) {
      size_t remaining = 0;
      {
        std::lock_guard<std::mutex> remaining_lk(m_);
        remaining = q_.size();
      }
      if (remaining == 0) {
        if (!logged_shutdown_flush_completed_) {
          logged_shutdown_flush_completed_ = true;
          RCLCPP_INFO(logger_,
            "Shutdown flush sent %zu measurement%s. All buffered measurements were uploaded.",
            lines, plural);
        } else {
          RCLCPP_INFO(logger_, "Shutdown flush sent %zu measurement%s.", lines, plural);
        }
      } else {
        const char* remain_plural = plural_suffix(remaining);
        RCLCPP_INFO(logger_,
          "Shutdown flush sent %zu measurement%s. %zu measurement%s remain queued.",
          lines, plural, remaining, remain_plural);
      }
    }

    batch.clear();
  }
}

bool InfluxWriter::post(const std::string& body){
  // initialize curl session
  CURL *curl = curl_easy_init();
  if(!curl) throw std::runtime_error("failed to initialize curl");

  // RAII cleanup, C++ free style
  struct CurlCleanup {
    CURL* h; curl_slist* headers;
    ~CurlCleanup(){ if (headers) curl_slist_free_all(headers); if (h) curl_easy_cleanup(h); }
  } cleanup{curl, nullptr};

  cleanup.headers = curl_slist_append(cleanup.headers, ("Authorization: Token " + token_).c_str());
  cleanup.headers = curl_slist_append(cleanup.headers, "Content-Type: text/plain; charset=utf-8");
  cleanup.headers = curl_slist_append(cleanup.headers, "Accept: application/json");

  std::string compressed_body;
  const std::string* payload = &body;
  if (use_gzip_) {
    try {
      compressed_body = compress_gzip_string(body);
    } catch (const std::exception& e) {
      throw InfluxError(std::string("Failed to gzip payload: ") + e.what());
    }
    payload = &compressed_body;
    cleanup.headers = curl_slist_append(cleanup.headers, "Content-Encoding: gzip");
  }

  // buffers
  std::string resp_body;
  std::string resp_ct;  // content-type
  char errbuf[CURL_ERROR_SIZE] = {0};

  curl_easy_setopt(curl, CURLOPT_URL, endpoint_.c_str());
  curl_easy_setopt(curl, CURLOPT_HTTPHEADER, cleanup.headers);
  curl_easy_setopt(curl, CURLOPT_POST, 1L);
  curl_easy_setopt(curl, CURLOPT_POSTFIELDS, payload->c_str());
  curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE_LARGE, static_cast<curl_off_t>(payload->size()));
  curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT_MS, 2000L);
  curl_easy_setopt(curl, CURLOPT_TIMEOUT_MS, 2000L);
  curl_easy_setopt(curl, CURLOPT_NOSIGNAL, 1L);
  curl_easy_setopt(curl, CURLOPT_TCP_KEEPALIVE, 1L);
  curl_easy_setopt(curl, CURLOPT_ACCEPT_ENCODING, "");
  curl_easy_setopt(curl, CURLOPT_USERAGENT, "influx-writer/1.0");
  curl_easy_setopt(curl, CURLOPT_ERRORBUFFER, errbuf);
  curl_easy_setopt(curl, CURLOPT_FAILONERROR, 1L); // make HTTP >=400 a CURLE_HTTP_RETURNED_ERROR

  // capture body and headers
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_cb);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &resp_body);
  curl_easy_setopt(curl, CURLOPT_HEADERFUNCTION, header_cb);
  curl_easy_setopt(curl, CURLOPT_HEADERDATA, &resp_ct);

  // perform
  CURLcode res = curl_easy_perform(curl);

  long http = 0;
  curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http);

  // handle errors
  if (res != CURLE_OK) {
    std::string msg = errbuf[0] ? errbuf : curl_easy_strerror(res);
    // include HTTP status if available; libcurl uses CURLE_HTTP_RETURNED_ERROR for >=400 with FAILONERROR
    if (res == CURLE_HTTP_RETURNED_ERROR) {
      const std::string parsed = parse_error_payload(http, resp_body, resp_ct);
      throw InfluxHttpError(
        parsed,
        http, res, resp_body
      );
    }
    throw InfluxNetworkError(msg, http, res, resp_body);
  }

  // success codes only
  if (http < 200 || http >= 300) {
    const std::string parsed = parse_error_payload(http, resp_body, resp_ct);
    throw InfluxHttpError(
      parsed,
      http, CURLE_OK, resp_body
    );
  }

  return true;
}

void InfluxWriter::set_last_error(std::string message) const {
  std::lock_guard<std::mutex> lk(error_mutex_);
  last_error_ = std::move(message);
}

} // namespace robot_influx_bridge
