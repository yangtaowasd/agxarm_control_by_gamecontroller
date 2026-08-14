// Copyright 2026 yang
// SPDX-License-Identifier: Apache-2.0

#include <fcntl.h>
#include <linux/input.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <functional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>

using namespace std::chrono_literals;

class KeyboardReader : public rclcpp::Node
{
public:
  KeyboardReader()
  : Node("keyboard_reader")
  {
    device_ = declare_parameter<std::string>("device", "/dev/input/event3");
    configure_keys();

    fd_ = open(device_.c_str(), O_RDONLY | O_NONBLOCK);
    if (fd_ < 0) {
      throw std::runtime_error(
              "Cannot open " + device_ + ": " + std::strerror(errno));
    }
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
    publisher_ = create_publisher<std_msgs::msg::Int32MultiArray>(topic_, qos);
    timer_ = create_wall_timer(period_, std::bind(&KeyboardReader::tick, this));
    RCLCPP_INFO(
      get_logger(), "Unified arm keyboard: %s -> %s (%zu keys)",
      device_.c_str(), topic_.c_str(), states_.size());
  }

  ~KeyboardReader() override
  {
    if (fd_ >= 0) {
      close(fd_);
    }
  }

private:
  void configure_keys()
  {
    topic_ = "/arm_keyboard_state";
    period_ = 5ms;
    bindings_ = {
      {KEY_1, 0}, {KEY_2, 1}, {KEY_3, 2}, {KEY_4, 3},
      {KEY_5, 4}, {KEY_6, 5}, {KEY_7, 6}, {KEY_A, 7},
      {KEY_D, 8}, {KEY_SPACE, 9}, {KEY_E, 10}, {KEY_P, 11},
      {KEY_W, 12}, {KEY_S, 13}, {KEY_Z, 14}, {KEY_X, 15},
      {KEY_I, 16}, {KEY_O, 23},
      {KEY_UP, 17}, {KEY_DOWN, 18}, {KEY_LEFT, 19}, {KEY_RIGHT, 20},
      {KEY_PAGEUP, 21}, {KEY_PAGEDOWN, 22},
    };
    states_.assign(24, 0);
  }

  void tick()
  {
    input_event event {};
    while (true) {
      const ssize_t bytes = read(fd_, &event, sizeof(event));
      if (bytes < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK) {
          RCLCPP_ERROR(get_logger(), "Keyboard read failed: %s", std::strerror(errno));
        }
        break;
      }
      if (bytes != static_cast<ssize_t>(sizeof(event))) {
        break;
      }
      if (event.type != EV_KEY) {
        continue;
      }
      const auto binding = bindings_.find(event.code);
      if (binding != bindings_.end()) {
        states_[binding->second] = event.value == 0 ? 0 : 1;
      }
    }
    std_msgs::msg::Int32MultiArray message;
    message.data = states_;
    publisher_->publish(message);
  }

  int fd_ {-1};
  std::string device_;
  std::string topic_;
  std::chrono::milliseconds period_ {1};
  std::unordered_map<uint16_t, std::size_t> bindings_;
  std::vector<int32_t> states_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<KeyboardReader>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("keyboard_reader"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
