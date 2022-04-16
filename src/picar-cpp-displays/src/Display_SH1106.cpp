#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

#include "lcdgfx.h"
#include <mutex>

class DistanceDisplay_SH1106 {
  public:
    DistanceDisplay_SH1106() {
      m_display.begin();
      clear();
      m_display.setFixedFont(ssd1306xled_font6x8);
      displayWait();
    }
    ~DistanceDisplay_SH1106() {
      clear();
    }
    DistanceDisplay_SH1106(const DistanceDisplay_SH1106&) = delete;
    DistanceDisplay_SH1106(DistanceDisplay_SH1106&&) = delete;
    DistanceDisplay_SH1106& operator=(const DistanceDisplay_SH1106&) = delete;
    DistanceDisplay_SH1106& operator=(DistanceDisplay_SH1106&&) = delete;

    void displayDistance(int distance) {
      std::scoped_lock lock{m_mutex};
      clear();
      m_display.printFixed(0,  8, "Distance:", STYLE_NORMAL);
      m_display.printFixed(0,  16, (std::to_string(distance)+" mm").data(), STYLE_NORMAL);
      lcd_delay(30);
    }

  private:
    void clear() {
      m_display.clear();
    }
    void displayWait() {
      clear();
      m_display.printFixed(0,  8, "Waiting for data...", STYLE_NORMAL);
    }
    DisplaySH1106_128x64_I2C m_display{-1};
    std::mutex m_mutex;
};

class DistanceDisplayNode : public rclcpp::Node {
  public:
    DistanceDisplayNode() : Node("Display_SH1106") {
      m_subscription = create_subscription<std_msgs::msg::Int32>(
        "distanceStable", 10, std::bind(&DistanceDisplayNode::distanceCallback, this, std::placeholders::_1)
      );
    }
  private:
    // Private methods
    void distanceCallback(const std_msgs::msg::Int32& msg) {
      RCLCPP_INFO(this->get_logger(), "Received distance: %d mm", msg.data);
      m_display.displayDistance(msg.data);
    }
    // Private members
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr m_subscription;
    DistanceDisplay_SH1106 m_display;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DistanceDisplayNode>());
  rclcpp::shutdown();
  return 0;
}
