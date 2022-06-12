#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

#include "lcdgfx.h"
#include "canvas/canvas.h"

#include <array>
#include <mutex>

namespace {
  template<size_t N>
  constexpr unsigned char* byteArrayToCharPtr(std::array<std::byte,N>& arr) {
    return reinterpret_cast<unsigned char*>(arr.data());
  }
  std::string formatMMtoCM(int millimeters) {
    if (millimeters==0) {return "0.0";}
    size_t nDigits{0};
    int powTen{1};
    while (millimeters >= powTen) {
      powTen *= 10;
      ++nDigits;
    }
    std::string ret(nDigits+1, char{0});
    std::sprintf(&ret[0], "%.1f", 0.1*millimeters);
    ret.append(" cm");
    return ret;
  }
}

class DistanceDisplay_SH1106 {
  public:
    DistanceDisplay_SH1106() {
      RCLCPP_INFO(get_logger(), "Creating display");
      m_display.begin();
      m_display.clear();
      m_display.setFixedFont(ssd1306xled_font6x8);
      m_canvas.setFixedFont(ssd1306xled_font6x8);
      displayWait();
    }
    ~DistanceDisplay_SH1106() {
      m_canvas.clear();
      m_display.clear();
      RCLCPP_INFO(get_logger(), "Destroying display");
    }
    DistanceDisplay_SH1106(const DistanceDisplay_SH1106&) = delete;
    DistanceDisplay_SH1106(DistanceDisplay_SH1106&&) = delete;
    DistanceDisplay_SH1106& operator=(const DistanceDisplay_SH1106&) = delete;
    DistanceDisplay_SH1106& operator=(DistanceDisplay_SH1106&&) = delete;

    void displayDistance(int distance) {
      std::scoped_lock lock{m_canvasMutex};
      m_canvas.clear();
      m_canvas.printFixed(0,  8, "Distance:", STYLE_NORMAL);
      m_canvas.printFixed(0,  16, formatMMtoCM(distance).data(), STYLE_NORMAL);
      m_display.drawCanvas(0, 0, m_canvas);
      lcd_delay(30);
    }

  private:
    // ---------- Private methods ----------------------------------------------
    void displayWait() {
      std::scoped_lock lock{m_canvasMutex};
      m_canvas.clear();
      m_canvas.printFixed(0,  8, "Waiting for data...", STYLE_NORMAL);
      m_display.drawCanvas(0, 0, m_canvas);
      lcd_delay(30);
    }
    rclcpp::Logger get_logger() {
      return rclcpp::get_logger("DistanceDisplay_SH1106");
    }
    // ---------- Private constants --------------------------------------------
    static constexpr unsigned int s_canvasWidth{128};
    static constexpr unsigned int s_canvasHeight{64};
    static constexpr unsigned int s_canvasBufferSize{s_canvasWidth*(s_canvasHeight/8)};
    // ---------- Private members ----------------------------------------------
    std::array<std::byte,s_canvasBufferSize> m_canvasBuffer{};
    NanoCanvas1 m_canvas{s_canvasWidth,s_canvasHeight,byteArrayToCharPtr(m_canvasBuffer)};
    DisplaySH1106_128x64_I2C m_display{-1};
    std::mutex m_canvasMutex;
};

class DistanceDisplayNode : public rclcpp::Node {
  public:
    DistanceDisplayNode() : Node("Display_SH1106") {
      m_subscription = create_subscription<std_msgs::msg::Int32>(
        "distanceStable", 10, std::bind(&DistanceDisplayNode::distanceCallback, this, std::placeholders::_1)
      );
    }
  private:
    // ---------- Private methods ----------------------------------------------
    void distanceCallback(const std_msgs::msg::Int32& msg) {
      RCLCPP_INFO(this->get_logger(), "Received distance: %d mm", msg.data);
      m_display.displayDistance(msg.data);
    }
    // ---------- Private members ----------------------------------------------
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
