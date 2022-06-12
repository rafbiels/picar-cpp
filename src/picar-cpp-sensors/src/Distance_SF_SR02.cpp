#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

#include <gpiod.hpp>

#include <chrono>
#include <iostream>
#include <list>
#include <memory>
#include <mutex>
#include <numeric>
#include <optional>
#include <sstream>
#include <string>
#include <thread>

namespace {
  const static std::string s_nodeName{"Distance_SF_SR02"};
  const static std::string s_chipName{"gpiochip0"};
  constexpr static unsigned int s_chipOffset{17};
  const static gpiod::line_request s_writeReq{
    .consumer=s_nodeName,
    .request_type=gpiod::line_request::DIRECTION_OUTPUT,
    .flags=0
  };
  const static gpiod::line_request s_readReq{
    .consumer=s_nodeName,
    .request_type=gpiod::line_request::EVENT_BOTH_EDGES,
    .flags=0
  };
}

class Chip_SF_SR02 {
  public:
    Chip_SF_SR02() : m_chip(s_chipName) {
      m_line = m_chip.get_line(s_chipOffset);
    }
    ~Chip_SF_SR02() {
      m_line.release();
      m_line.reset();
    }
    Chip_SF_SR02(const Chip_SF_SR02& other) = delete;
    Chip_SF_SR02(Chip_SF_SR02&& other) = delete;
    Chip_SF_SR02& operator=(const Chip_SF_SR02& other) = delete;
    Chip_SF_SR02& operator=(Chip_SF_SR02&& other) = delete;

    void setLogger(rclcpp::Logger&& logger) {m_logger = logger;}

    int readDistance() {
      gpiod::line_event ev1{std::chrono::nanoseconds{0},0,gpiod::line{}};
      gpiod::line_event ev2{std::chrono::nanoseconds{0},0,gpiod::line{}};
      { // locked scope
        std::scoped_lock lock(m_chipMutex);
        m_line.release();
        m_line.request(s_writeReq, 1);
        std::this_thread::sleep_for(m_pulseLength);
        m_line.release();
        m_line.request(s_readReq);

        if (!m_line.event_wait(m_readTimeout)) {
          RCLCPP_DEBUG(getLogger(), "Event wait timed out");
          m_line.release();
          return -1;
        }
        ev1 = m_line.event_read();

        if (!m_line.event_wait(m_readTimeout)) {
          RCLCPP_DEBUG(getLogger(), "Event wait timed out");
          m_line.release();
          return -1;
        }
        ev2 = m_line.event_read();

        m_line.release();
      }

      if (ev1.event_type != gpiod::line_event::RISING_EDGE || ev2.event_type != gpiod::line_event::FALLING_EDGE) {
        RCLCPP_DEBUG(getLogger(), "Wrong event types:\n%s\n%s", printEvent(ev1).c_str(), printEvent(ev2).c_str());
        return -1;
      }

      return timeToDistanceMillimeters(ev1.timestamp, ev2.timestamp);
    }

  private:
    // Private methods
    int timeToDistanceMillimeters(std::chrono::nanoseconds t1, std::chrono::nanoseconds t2) {
      long long int diffNano = (t2-t1).count();
      static constexpr long long int soundSpeed{340};
      static constexpr long long int nanoToMilli{1000000};
      int soundTravelled = static_cast<int>(diffNano*soundSpeed/nanoToMilli);
      int rawDistance = soundTravelled / 2; // Sound travels there and back
      static constexpr int correction{50}; // Measurements of my sensor seem to be off by this value, don't know why
      return rawDistance + correction;
    }
    rclcpp::Logger& getLogger() {
      if (not m_logger.has_value()) {
        m_logger = rclcpp::get_logger("Chip_SF_SR02");
      }
      return m_logger.value();
    }

    std::string printEvent(const gpiod::line_event& event) {
      std::ostringstream ss;
      if (event.event_type == gpiod::line_event::RISING_EDGE) {
        ss << " RISING EDGE ";
      } else if (event.event_type == gpiod::line_event::FALLING_EDGE) {
        ss << "FALLING EDGE ";
      } else {
        ss << "Invalid event type ";
      }
      ss << std::chrono::duration_cast<std::chrono::seconds>(event.timestamp).count();
      ss << ".";
      ss << event.timestamp.count() % 1000000000;
      ss << " line: " << event.source.offset();
      return ss.str();
    }

    // Private members
    std::optional<rclcpp::Logger> m_logger;
    gpiod::chip m_chip;
    gpiod::line m_line;
    std::mutex m_chipMutex;
    std::chrono::microseconds m_pulseLength{20};
    std::chrono::milliseconds m_readTimeout{50};
};

class DistancePublisher : public rclcpp::Node {
  public:
    DistancePublisher() : Node("Distance_SF_SR02") {
      m_publisher = create_publisher<std_msgs::msg::Int32>("distance", 10);
      m_publisherStable = create_publisher<std_msgs::msg::Int32>("distanceStable", 10);
      m_timer = create_wall_timer(m_interval, std::bind(&DistancePublisher::timer_callback, this));
      m_chip.setLogger(get_logger());

      auto ret = rcutils_logging_set_logger_level(
        get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
      if (ret != RCUTILS_RET_OK) {
        RCLCPP_ERROR(get_logger(), "Error setting severity: %s", rcutils_get_error_string().str);
        rcutils_reset_error();
      }
    }

  private:
    // Private methods
    void timer_callback() {
      int distance = m_chip.readDistance();
      if (distance<0) {
        RCLCPP_DEBUG(get_logger(), "Distance not read, not publishing");
        return;
      }
      auto message = std_msgs::msg::Int32();
      message.data = distance;
      m_publisher->publish(message);
      auto messageStable = std_msgs::msg::Int32();
      messageStable.data = stableValue(distance);
      m_publisherStable->publish(messageStable);
      RCLCPP_INFO(get_logger(), "Published distance, insta = %d mm, stable = %d mm", distance, messageStable.data);
    }

    int stableValue(int instaValue) {
      while (m_recentValues.size()>=m_maxRecentValues) {
        m_recentValues.pop_back();
      }
      m_recentValues.push_front(instaValue);
      return std::accumulate(m_recentValues.begin(),m_recentValues.end(),int{0}) / m_recentValues.size();
    }

    // Private members
    rclcpp::TimerBase::SharedPtr m_timer;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr m_publisher;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr m_publisherStable;
    std::chrono::milliseconds m_interval{80};
    std::list<int> m_recentValues;
    size_t m_maxRecentValues{10};
    Chip_SF_SR02 m_chip;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DistancePublisher>());
  rclcpp::shutdown();
  return 0;
}
