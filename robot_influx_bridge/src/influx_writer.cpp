#include <curl/curl.h>

#include <algorithm>
#include <chrono>
#include <cctype>
#include <sstream>
#include <stdexcept>
#include <thread>

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
}  // namespace

using namespace std::chrono_literals;
namespace robot_influx_bridge {

InfluxWriter::InfluxWriter(const rclcpp::Logger& logger,
    const std::string& url,const std::string& org,
    const std::string& bucket,const std::string& token,size_t max_batch,size_t max_queue,
    std::chrono::milliseconds flush)
: token_(token), max_batch_(max_batch), max_queue_(max_queue), flush_(flush), logger_(logger)
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
  std::vector<std::string> batch; batch.reserve(max_batch_);
  while(!stop_){
    std::unique_lock<std::mutex> lk(m_);
    cv_.wait_for(lk, flush_, [&]{ return stop_ || q_.size() >= max_batch_ || !q_.empty(); });
    while(!q_.empty() && batch.size() < max_batch_){
      batch.push_back(std::move(q_.front())); q_.pop();
    }
    lk.unlock();
    if(batch.empty()) continue;
    std::string body;
    body.reserve(batch.size() * 64);
    for(size_t i=0;i<batch.size();++i){ body += batch[i]; body.push_back('\n'); }
    // retry with basic backoff
    for(int attempt=0; attempt<5; ++attempt){
      try {
        if(post(body)) {
          set_last_error("");
          const bool had_error = error_since_last_success_.exchange(false, std::memory_order_relaxed);
          const size_t lines = batch.size();
          const char* plural = lines == 1 ? "" : "s";
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
      } catch(const InfluxError& e) {
        error_since_last_success_.store(true, std::memory_order_relaxed);
        const std::string reason = describe_error(e);
        RCLCPP_ERROR(logger_, "InfluxDB write attempt %d/%d failed: %s",
          attempt + 1, 5, reason.c_str());
        set_last_error(reason);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(std::min(5000, 50 * (1<<attempt))));
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

  // buffers
  std::string resp_body;
  std::string resp_ct;  // content-type
  char errbuf[CURL_ERROR_SIZE] = {0};

  curl_easy_setopt(curl, CURLOPT_URL, endpoint_.c_str());
  curl_easy_setopt(curl, CURLOPT_HTTPHEADER, cleanup.headers);
  curl_easy_setopt(curl, CURLOPT_POST, 1L);
  curl_easy_setopt(curl, CURLOPT_POSTFIELDS, body.c_str());
  curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE_LARGE, static_cast<curl_off_t>(body.size()));
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
      // best-effort parse of JSON error
      std::string parsed;
      if (!resp_body.empty() && resp_ct.find("application/json") != std::string::npos) {
        const auto code = extract_json_string(resp_body, "code");
        const auto m = extract_json_string(resp_body, "message");
        if (!m.empty()) parsed = m;
        if (!code.empty()) parsed = code + ": " + parsed;
      }
      if (parsed.empty()) parsed = trim(resp_body);
      throw InfluxHttpError(
        parsed.empty() ? ("HTTP " + std::to_string(http)) : parsed,
        http, res, resp_body
      );
    }
    throw InfluxNetworkError(msg, http, res, resp_body);
  }

  // success codes only
  if (http < 200 || http >= 300) {
    std::string parsed;
    if (!resp_body.empty() && resp_ct.find("application/json") != std::string::npos) {
      const auto code = extract_json_string(resp_body, "code");
      const auto m = extract_json_string(resp_body, "message");
      if (!m.empty()) parsed = m;
      if (!code.empty()) parsed = code + ": " + parsed;
    }
    if (parsed.empty()) parsed = trim(resp_body);
    throw InfluxHttpError(
      parsed.empty() ? ("HTTP " + std::to_string(http)) : parsed,
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
