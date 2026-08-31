#include "byd_event_rosbag_recorder/event_recorder_core.hpp"

#include <byd_vehicle_msgs/msg/active_event.hpp>
#include <byd_vehicle_msgs/msg/event_trigger.hpp>
#include <byd_vehicle_msgs/msg/recorder_status.hpp>
#include <byd_vehicle_msgs/srv/trigger_event.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_compression/compression_options.hpp>
#include <rosbag2_compression/sequential_compression_writer.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/storage_options.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <chrono>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <mutex>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace fs = std::filesystem;
using namespace std::chrono_literals;

namespace byd_event_rosbag_recorder {

struct BufferedMessage {
  int64_t timestamp_ns;
  std::string topic;
  std::string type;
  std::shared_ptr<rclcpp::SerializedMessage> data;
};

struct EventRecord {
  int64_t timestamp_ns;
  std::string id;
  std::string type;
  uint8_t severity;
  std::string description;
  std::string source;
};

class EventRosbagRecorderNode : public rclcpp::Node {
public:
  EventRosbagRecorderNode()
      : Node("event_rosbag_recorder_node"),
        pre_ns_(seconds_to_ns(
            declare_parameter("recording.pre_trigger_seconds", 30.0))),
        post_ns_(seconds_to_ns(
            declare_parameter("recording.post_trigger_seconds", 30.0))),
        max_duration_ns_(seconds_to_ns(
            declare_parameter("recording.max_event_duration_seconds", 600.0))),
        queue_capacity_(static_cast<std::size_t>(
            declare_parameter("recording.queue_capacity_messages", 10000))),
        segment_seconds_(
            declare_parameter("recording.segment_seconds", 5.0)),
        capture_(pre_ns_, post_ns_, max_duration_ns_),
        output_directory_(
            declare_parameter("storage.output_directory",
                              "/home/nvidia/autoware/log")),
        temporary_directory_(declare_parameter(
            "storage.temporary_directory", output_directory_ + "/.buffer")),
        minimum_free_space_gb_(
            declare_parameter("storage.minimum_free_space_gb", 20.0)),
        maximum_event_storage_gb_(
            declare_parameter("storage.maximum_event_storage_gb", 200.0)),
        compression_(declare_parameter("storage.compression", "zstd")),
        exact_topics_(
            declare_parameter<std::vector<std::string>>(
                "topics.names", std::vector<std::string>{})),
        diagnostic_filter_(
            static_cast<uint8_t>(declare_parameter("diagnostics.min_level", 2)),
            declare_parameter<std::vector<std::string>>(
                "diagnostics.include_names", {".*"}),
            declare_parameter<std::vector<std::string>>(
                "diagnostics.exclude_names", {"^event_rosbag_recorder($|:)"})),
        diagnostic_trigger_on_transition_(declare_parameter(
            "diagnostics.trigger_on_transition", true)) {
    declare_parameter("storage.cleanup_policy", "oldest_first");
    const auto storage_id = declare_parameter("storage.storage_id", "mcap");
    if (pre_ns_ < 0 || post_ns_ < 0 || output_directory_.empty() ||
        temporary_directory_.empty() || !std::isfinite(segment_seconds_) ||
        segment_seconds_ < 0.0 || queue_capacity_ == 0) {
      throw std::invalid_argument(
          "invalid recording duration, queue capacity, or storage directory");
    }
    if (storage_id != "mcap") {
      throw std::invalid_argument("storage.storage_id must be mcap");
    }
    compile_regex_parameter("topics.include_regex", include_topics_);
    compile_regex_parameter("topics.exclude_regex", exclude_topics_);
    fs::create_directories(output_directory_);
    fs::create_directories(temporary_directory_);
    if (fs::equivalent(output_directory_, temporary_directory_)) {
      throw std::invalid_argument(
          "storage.temporary_directory must differ from output_directory");
    }
    quarantine_incomplete_events();

    event_sub_ = create_subscription<byd_vehicle_msgs::msg::EventTrigger>(
        "/system/event_trigger", rclcpp::QoS(10).reliable(),
        [this](const byd_vehicle_msgs::msg::EventTrigger::SharedPtr msg) {
          EventRecord event{now().nanoseconds(), msg->event_id,
                            msg->event_type,     msg->severity,
                            msg->description,    "topic"};
          trigger(std::move(event));
        });
    diagnostics_sub_ =
        create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
            "/diagnostics", rclcpp::QoS(50),
            [this](const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg) {
              for (const auto &status : msg->status) {
                if (diagnostic_filter_.should_trigger(
                        status.name, status.level,
                        diagnostic_trigger_on_transition_)) {
                  EventRecord event{now().nanoseconds(), "",
                                    status.name,         status.level,
                                    status.message,      "diagnostics"};
                  trigger(std::move(event));
                } else if (diagnostic_filter_.is_abnormal(status.name,
                                                          status.level)) {
                  std::lock_guard<std::mutex> lock(mutex_);
                  if (capture_.active()) {
                    capture_.trigger(now().nanoseconds());
                  }
                }
              }
            });

    status_pub_ = create_publisher<byd_vehicle_msgs::msg::RecorderStatus>(
        "/event_rosbag_recorder/status",
        rclcpp::QoS(1).reliable().transient_local());
    active_pub_ = create_publisher<byd_vehicle_msgs::msg::ActiveEvent>(
        "/event_rosbag_recorder/active_event",
        rclcpp::QoS(1).reliable().transient_local());
    diagnostic_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
        "/diagnostics", 10);
    trigger_service_ = create_service<byd_vehicle_msgs::srv::TriggerEvent>(
        "/event_rosbag_recorder/trigger",
        [this](
            const std::shared_ptr<byd_vehicle_msgs::srv::TriggerEvent::Request>
                request,
            std::shared_ptr<byd_vehicle_msgs::srv::TriggerEvent::Response>
                response) {
          const auto id = make_event_id();
          trigger(EventRecord{now().nanoseconds(), id, request->event_type,
                              request->severity, request->description,
                              "service"});
          response->accepted = true;
          response->event_id = id;
          response->message = "event accepted";
        });

    discovery_timer_ = create_wall_timer(1s, [this]() { discover_topics(); });
    maintenance_timer_ = create_wall_timer(200ms, [this]() { maintain(); });
    discover_topics();
  }

private:
  static int64_t seconds_to_ns(double seconds) {
    return static_cast<int64_t>(seconds * 1'000'000'000.0);
  }

  void compile_regex_parameter(const std::string &name,
                               std::vector<std::regex> &output) {
    for (const auto &expression :
         declare_parameter<std::vector<std::string>>(
             name, std::vector<std::string>{})) {
      output.emplace_back(expression);
    }
  }

  void discover_topics() {
    for (const auto &[topic, types] : get_topic_names_and_types()) {
      if (!topic_selected(topic, exact_topics_, include_topics_,
                          exclude_topics_)) {
        continue;
      }
      if (generic_subscriptions_.count(topic) != 0) {
        continue;
      }
      if (types.size() != 1) {
        topic_errors_[topic] = "topic must advertise exactly one type";
        continue;
      }
      const auto type = types.front();
      try {
        auto subscription = create_generic_subscription(
            topic, type, rclcpp::SensorDataQoS(),
            [this, topic,
             type](std::shared_ptr<rclcpp::SerializedMessage> message) {
              std::lock_guard<std::mutex> lock(mutex_);
              if (messages_.size() >= queue_capacity_) {
                messages_.pop_front();
                ++dropped_message_count_;
              }
              messages_.push_back(
                  BufferedMessage{now().nanoseconds(), topic, type, message});
            });
        generic_subscriptions_[topic] = subscription;
        topic_types_[topic] = type;
        topic_errors_.erase(topic);
      } catch (const std::exception &error) {
        topic_errors_[topic] = error.what();
      }
    }
  }

  void trigger(EventRecord event) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (event.id.empty()) {
      event.id = make_event_id();
    }
    capture_.trigger(event.timestamp_ns);
    events_.push_back(std::move(event));
  }

  void maintain() {
    const auto now_ns = now().nanoseconds();
    bool should_finalize = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const auto retain_from =
          capture_.active() ? capture_.start_ns() : now_ns - pre_ns_;
      while (!messages_.empty() &&
             messages_.front().timestamp_ns < retain_from) {
        messages_.pop_front();
      }
      should_finalize = capture_.expired(now_ns);
    }
    if (should_finalize) {
      finalize_capture();
    }
    publish_status();
  }

  void finalize_capture() {
    std::vector<BufferedMessage> selected;
    std::vector<EventRecord> events;
    int64_t start_ns;
    int64_t end_ns;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!capture_.active()) {
        return;
      }
      state_ = byd_vehicle_msgs::msg::RecorderStatus::STATE_FINALIZING;
      start_ns = capture_.start_ns();
      end_ns = capture_.end_ns();
      events = events_;
      for (const auto &message : messages_) {
        if (message.timestamp_ns >= start_ns &&
            message.timestamp_ns <= end_ns) {
          selected.push_back(message);
        }
      }
    }

    try {
      ensure_capacity();
      const auto base_name =
          unique_output_name(make_output_name(events.front()));
      const fs::path staging = fs::path(temporary_directory_) / base_name;
      const fs::path completed = fs::path(output_directory_) / base_name;
      rosbag2_storage::StorageOptions options;
      options.uri = staging.string();
      options.storage_id = "mcap";
      options.max_bagfile_duration = segment_seconds_ == 0.0
                                         ? 0
                                         : static_cast<uint64_t>(
                                               std::ceil(segment_seconds_));
      std::unique_ptr<rosbag2_cpp::Writer> writer;
      if (!compression_.empty() && compression_ != "none") {
        rosbag2_compression::CompressionOptions compression_options;
        compression_options.compression_format = compression_;
        compression_options.compression_mode =
            rosbag2_compression::CompressionMode::FILE;
        compression_options.compression_queue_size = 1;
        compression_options.compression_threads = 1;
        writer = std::make_unique<rosbag2_cpp::Writer>(
            std::make_unique<
                rosbag2_compression::SequentialCompressionWriter>(
                compression_options));
      } else {
        writer = std::make_unique<rosbag2_cpp::Writer>();
      }
      writer->open(options);
      for (const auto &message : selected) {
        writer->write(message.data, message.topic, message.type,
                      rclcpp::Time(message.timestamp_ns));
      }
      writer->close();
      write_manifest(staging / "event.json", start_ns, end_ns, events,
                     selected.size());
      promote_event(staging, completed, base_name);

      std::lock_guard<std::mutex> lock(mutex_);
      capture_.reset();
      events_.clear();
      last_error_.clear();
      state_ = byd_vehicle_msgs::msg::RecorderStatus::STATE_BUFFERING;
    } catch (const std::exception &error) {
      std::lock_guard<std::mutex> lock(mutex_);
      last_error_ = error.what();
      state_ = byd_vehicle_msgs::msg::RecorderStatus::STATE_DEGRADED;
      capture_.reset();
      events_.clear();
    }
  }

  static uintmax_t directory_size(const fs::path &directory) {
    uintmax_t bytes = 0;
    std::error_code error;
    for (fs::recursive_directory_iterator iterator(
             directory, fs::directory_options::skip_permission_denied, error),
         end;
         iterator != end; iterator.increment(error)) {
      if (!error && iterator->is_regular_file(error)) {
        bytes += iterator->file_size(error);
      }
      error.clear();
    }
    return bytes;
  }

  bool is_completed_event(const fs::directory_entry &entry) const {
    if (!entry.is_directory()) {
      return false;
    }
    const auto name = entry.path().filename().string();
    return name.find(".inprogress") == std::string::npos &&
           name.find(".corrupt") == std::string::npos &&
           entry.path().parent_path() == fs::path(output_directory_) &&
           fs::exists(entry.path() / "metadata.yaml") &&
           fs::exists(entry.path() / "event.json");
  }

  void ensure_capacity() {
    std::vector<fs::directory_entry> completed;
    uintmax_t event_bytes = 0;
    for (const auto &entry : fs::directory_iterator(output_directory_)) {
      if (is_completed_event(entry)) {
        completed.push_back(entry);
        event_bytes += directory_size(entry.path());
      }
    }
    std::sort(completed.begin(), completed.end(),
              [](const auto &left, const auto &right) {
                return left.last_write_time() < right.last_write_time();
              });
    const auto max_bytes =
        static_cast<uintmax_t>(maximum_event_storage_gb_ * 1'000'000'000.0);
    auto free_gb = [this]() {
      return static_cast<double>(fs::space(output_directory_).available) / 1e9;
    };
    for (const auto &entry : completed) {
      if (event_bytes <= max_bytes && free_gb() >= minimum_free_space_gb_) {
        break;
      }
      const auto bytes = directory_size(entry.path());
      fs::remove_all(entry.path());
      event_bytes = bytes > event_bytes ? 0 : event_bytes - bytes;
    }
    if (free_gb() < minimum_free_space_gb_) {
      throw std::runtime_error(
          "free disk space is below configured safety margin");
    }
  }

  void quarantine_incomplete_events() {
    for (const auto &entry : fs::directory_iterator(output_directory_)) {
      if (!entry.is_directory()) {
        continue;
      }
      const auto name = entry.path().filename().string();
      if (name.size() < 11 || name.substr(name.size() - 11) != ".inprogress") {
        continue;
      }
      fs::path target = entry.path();
      target.replace_extension(".corrupt");
      for (unsigned suffix = 1; fs::exists(target); ++suffix) {
        target = fs::path(entry.path().string() + ".corrupt." +
                          std::to_string(suffix));
      }
      fs::rename(entry.path(), target);
    }
    for (const auto &entry : fs::directory_iterator(temporary_directory_)) {
      if (!entry.is_directory() ||
          entry.path().filename().string().find(".corrupt") !=
              std::string::npos) {
        continue;
      }
      fs::path target(entry.path().string() + ".corrupt");
      for (unsigned suffix = 1; fs::exists(target); ++suffix) {
        target = fs::path(entry.path().string() + ".corrupt." +
                          std::to_string(suffix));
      }
      fs::rename(entry.path(), target);
    }
  }

  void promote_event(const fs::path &staging, const fs::path &completed,
                     const std::string &base_name) const {
    try {
      fs::rename(staging, completed);
      return;
    } catch (const fs::filesystem_error &error) {
      if (error.code() != std::errc::cross_device_link) {
        throw;
      }
    }

    const fs::path output_staging =
        fs::path(output_directory_) / (base_name + ".inprogress");
    fs::copy(staging, output_staging, fs::copy_options::recursive);
    fs::rename(output_staging, completed);
    fs::remove_all(staging);
  }

  std::string unique_output_name(const std::string &candidate) const {
    if (!fs::exists(fs::path(output_directory_) / candidate) &&
        !fs::exists(fs::path(output_directory_) /
                    (candidate + ".inprogress")) &&
        !fs::exists(fs::path(temporary_directory_) / candidate)) {
      return candidate;
    }
    for (unsigned suffix = 1;; ++suffix) {
      const auto value = candidate + "_" + std::to_string(suffix);
      if (!fs::exists(fs::path(output_directory_) / value) &&
          !fs::exists(fs::path(output_directory_) /
                      (value + ".inprogress")) &&
          !fs::exists(fs::path(temporary_directory_) / value)) {
        return value;
      }
    }
  }

  static std::string json_escape(const std::string &input) {
    std::string output;
    for (const char value : input) {
      if (value == '"' || value == '\\') {
        output.push_back('\\');
      }
      output.push_back(value == '\n' ? ' ' : value);
    }
    return output;
  }

  static void write_manifest(const fs::path &path, int64_t start_ns,
                             int64_t end_ns,
                             const std::vector<EventRecord> &events,
                             std::size_t message_count) {
    std::ofstream stream(path);
    if (!stream) {
      throw std::runtime_error("failed to create event manifest");
    }
    stream << "{\n  \"schema_version\": 1,\n  \"window_start_ns\": " << start_ns
           << ",\n  \"window_end_ns\": " << end_ns
           << ",\n  \"message_count\": " << message_count
           << ",\n  \"events\": [\n";
    for (std::size_t index = 0; index < events.size(); ++index) {
      const auto &event = events[index];
      stream << "    {\"timestamp_ns\": " << event.timestamp_ns
             << ", \"event_id\": \"" << json_escape(event.id)
             << "\", \"event_type\": \"" << json_escape(event.type)
             << "\", \"severity\": " << static_cast<unsigned>(event.severity)
             << ", \"description\": \"" << json_escape(event.description)
             << "\", \"source\": \"" << json_escape(event.source) << "\"}"
             << (index + 1 == events.size() ? "\n" : ",\n");
    }
    stream << "  ]\n}\n";
    stream.flush();
    if (!stream) {
      throw std::runtime_error("failed to flush event manifest");
    }
  }

  static std::string sanitize(std::string value) {
    for (char &character : value) {
      if (!std::isalnum(static_cast<unsigned char>(character)) &&
          character != '-' && character != '_') {
        character = '_';
      }
    }
    return value.empty() ? "event" : value;
  }

  std::string make_output_name(const EventRecord &event) const {
    const auto system_now = std::chrono::system_clock::now();
    const auto value = std::chrono::system_clock::to_time_t(system_now);
    std::tm time{};
    gmtime_r(&value, &time);
    std::ostringstream stream;
    stream << std::put_time(&time, "%Y%m%d_%H%M%S") << "_"
           << static_cast<unsigned>(event.severity) << "_"
           << sanitize(event.type);
    return stream.str();
  }

  std::string make_event_id() const {
    std::ostringstream stream;
    stream << std::hex << now().nanoseconds();
    return stream.str();
  }

  void publish_status() {
    byd_vehicle_msgs::msg::RecorderStatus status;
    byd_vehicle_msgs::msg::ActiveEvent active;
    diagnostic_msgs::msg::DiagnosticArray diagnostics;
    diagnostic_msgs::msg::DiagnosticStatus diagnostic;
    status.header.stamp = now();
    active.header = status.header;
    diagnostics.header = status.header;
    diagnostic.name = "event_rosbag_recorder";
    diagnostic.hardware_id = "storage";
    {
      std::lock_guard<std::mutex> lock(mutex_);
      status.state = capture_.active() && state_ == status.STATE_BUFFERING
                         ? status.STATE_CAPTURING
                         : state_;
      status.state_message = last_error_.empty() ? "ready" : last_error_;
      status.selected_topic_count = generic_subscriptions_.size();
      status.missing_topic_count = topic_errors_.size();
      status.dropped_message_count = dropped_message_count_;
      status.active_event_id = events_.empty() ? "" : events_.front().id;
      if (!messages_.empty()) {
        status.buffer_coverage_seconds =
            static_cast<double>(messages_.back().timestamp_ns -
                                messages_.front().timestamp_ns) /
            1e9;
      }
      active.active = capture_.active();
      active.primary_event_id = status.active_event_id;
      active.window_start = rclcpp::Time(capture_.start_ns());
      active.expected_window_end = rclcpp::Time(capture_.end_ns());
      active.merged_event_count = capture_.trigger_count();
      diagnostic.level = state_ == status.STATE_DEGRADED
                             ? diagnostic_msgs::msg::DiagnosticStatus::ERROR
                             : diagnostic_msgs::msg::DiagnosticStatus::OK;
      diagnostic.message = status.state_message;
    }
    try {
      const auto info = fs::space(output_directory_);
      status.free_space_gb = static_cast<double>(info.available) / 1e9;
    } catch (const std::exception &) {
      status.free_space_gb = 0.0;
    }
    diagnostics.status.push_back(diagnostic);
    status_pub_->publish(status);
    active_pub_->publish(active);
    diagnostic_pub_->publish(diagnostics);
  }

  const int64_t pre_ns_;
  const int64_t post_ns_;
  const int64_t max_duration_ns_;
  const std::size_t queue_capacity_;
  const double segment_seconds_;
  CaptureWindow capture_;
  std::string output_directory_;
  std::string temporary_directory_;
  double minimum_free_space_gb_;
  double maximum_event_storage_gb_;
  std::string compression_;
  std::vector<std::string> exact_topics_;
  std::vector<std::regex> include_topics_;
  std::vector<std::regex> exclude_topics_;
  DiagnosticTransitionFilter diagnostic_filter_;
  const bool diagnostic_trigger_on_transition_;

  std::mutex mutex_;
  std::deque<BufferedMessage> messages_;
  std::vector<EventRecord> events_;
  std::unordered_map<std::string, std::string> topic_types_;
  std::unordered_map<std::string, std::string> topic_errors_;
  std::unordered_map<std::string, rclcpp::GenericSubscription::SharedPtr>
      generic_subscriptions_;
  uint8_t state_{byd_vehicle_msgs::msg::RecorderStatus::STATE_BUFFERING};
  std::string last_error_;
  uint64_t dropped_message_count_{0};

  rclcpp::Subscription<byd_vehicle_msgs::msg::EventTrigger>::SharedPtr
      event_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
      diagnostics_sub_;
  rclcpp::Publisher<byd_vehicle_msgs::msg::RecorderStatus>::SharedPtr
      status_pub_;
  rclcpp::Publisher<byd_vehicle_msgs::msg::ActiveEvent>::SharedPtr active_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
      diagnostic_pub_;
  rclcpp::Service<byd_vehicle_msgs::srv::TriggerEvent>::SharedPtr
      trigger_service_;
  rclcpp::TimerBase::SharedPtr discovery_timer_;
  rclcpp::TimerBase::SharedPtr maintenance_timer_;
};

} // namespace byd_event_rosbag_recorder

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(
      std::make_shared<byd_event_rosbag_recorder::EventRosbagRecorderNode>());
  rclcpp::shutdown();
  return 0;
}
