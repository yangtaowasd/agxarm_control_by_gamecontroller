// Copyright 2026 yang
// SPDX-License-Identifier: Apache-2.0

#include <fcntl.h>
#include <linux/input.h>
#include <unistd.h>
#include <X11/Xlib.h>
#include <X11/keysym.h>

#undef None

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

    if (device_ == "x11") {
      configure_x11();
    } else {
      fd_ = open(device_.c_str(), O_RDONLY | O_NONBLOCK);
      if (fd_ < 0) {
        throw std::runtime_error(
                "Cannot open " + device_ + ": " + std::strerror(errno));
      }
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
    if (x_display_ != nullptr) {
      XCloseDisplay(x_display_);
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
      {KEY_I, 16}, {KEY_O, 23}, {KEY_H, 24},
      {KEY_UP, 17}, {KEY_DOWN, 18}, {KEY_LEFT, 19}, {KEY_RIGHT, 20},
      {KEY_PAGEUP, 21}, {KEY_PAGEDOWN, 22},
    };
    states_.assign(25, 0);
  }

  void configure_x11()
  {
    x_display_ = XOpenDisplay(nullptr);
    if (x_display_ == nullptr) {
      const char * display_name = XDisplayName(nullptr);
      throw std::runtime_error(
              "Cannot open X11 display '" + std::string(display_name) +
              "'. Check DISPLAY and XAUTHORITY.");
    }

    const std::vector<std::pair<KeySym, std::size_t>> keysyms = {
      {XK_1, 0}, {XK_2, 1}, {XK_3, 2}, {XK_4, 3},
      {XK_5, 4}, {XK_6, 5}, {XK_7, 6}, {XK_a, 7},
      {XK_d, 8}, {XK_space, 9}, {XK_e, 10}, {XK_p, 11},
      {XK_w, 12}, {XK_s, 13}, {XK_z, 14}, {XK_x, 15},
      {XK_i, 16}, {XK_o, 23}, {XK_h, 24},
      {XK_Up, 17}, {XK_Down, 18}, {XK_Left, 19}, {XK_Right, 20},
      {XK_Page_Up, 21}, {XK_Page_Down, 22},
    };
    for (const auto & [keysym, state_index] : keysyms) {
      const KeyCode keycode = XKeysymToKeycode(x_display_, keysym);
      if (keycode == 0) {
        throw std::runtime_error("X11 key mapping is incomplete");
      }
      x11_bindings_.emplace_back(keycode, state_index);
    }
  }

  void read_evdev_state()
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
  }

  void read_x11_state()
  {
    char keymap[32] {};
    XQueryKeymap(x_display_, keymap);
    for (const auto & [keycode, state_index] : x11_bindings_) {
      const auto byte = static_cast<unsigned int>(keycode) / 8U;
      const auto bit = static_cast<unsigned int>(keycode) % 8U;
      states_[state_index] =
        (static_cast<unsigned char>(keymap[byte]) & (1U << bit)) != 0U ? 1 : 0;
    }
  }

  void tick()
  {
    if (x_display_ != nullptr) {
      read_x11_state();
    } else {
      read_evdev_state();
    }
    std_msgs::msg::Int32MultiArray message;
    message.data = states_;
    publisher_->publish(message);
  }

  int fd_ {-1};
  Display * x_display_ {nullptr};
  std::string device_;
  std::string topic_;
  std::chrono::milliseconds period_ {1};
  std::unordered_map<uint16_t, std::size_t> bindings_;
  std::vector<std::pair<KeyCode, std::size_t>> x11_bindings_;
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
