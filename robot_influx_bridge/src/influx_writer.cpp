#include <curl/curl.h>

#include <chrono>
#include <thread>

#include <robot_influx_bridge/influx_writer.hpp>

using namespace std::chrono_literals;
namespace robot_influx_bridge {

InfluxWriter::InfluxWriter(const std::string& url,const std::string& org,
    const std::string& bucket,const std::string& token,size_t max_batch,size_t max_queue,
    std::chrono::milliseconds flush)
: token_(token), max_batch_(max_batch), max_queue_(max_queue), flush_(flush)
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
  if(q_.size() >= max_queue_) return false;
  q_.push(std::move(line));
  cv_.notify_one();
  return true;
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
      if(post(body)) break;
      std::this_thread::sleep_for(std::chrono::milliseconds(std::min(5000, 50 * (1<<attempt))));
    }
    batch.clear();
  }
}

bool InfluxWriter::post(const std::string& body){
  CURL *curl = curl_easy_init();
  if(!curl) return false;
  struct curl_slist *headers=nullptr;
  headers = curl_slist_append(headers, ("Authorization: Token " + token_).c_str());
  headers = curl_slist_append(headers, "Content-Type: text/plain; charset=utf-8");
  curl_easy_setopt(curl, CURLOPT_URL, endpoint_.c_str());
  curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
  curl_easy_setopt(curl, CURLOPT_POSTFIELDS, body.c_str());
  curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, body.size());
  curl_easy_setopt(curl, CURLOPT_TIMEOUT_MS, 2000L);
  CURLcode res = curl_easy_perform(curl);
  long code = 0; curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &code);
  curl_slist_free_all(headers);
  curl_easy_cleanup(curl);
  return (res == CURLE_OK) && (code >= 200 && code < 300);
}

} // namespace robot_influx_bridge
