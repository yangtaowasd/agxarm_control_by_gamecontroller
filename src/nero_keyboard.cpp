#include <fcntl.h>
#include <linux/input.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <functional>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>

using namespace std::chrono_literals;

class NeroKeyboardReader : public rclcpp::Node
{
public:
  static constexpr std::size_t KEY_COUNT = 11;

  NeroKeyboardReader()
  : Node("nero_keyboard_reader"), keyboard_fd_(-1)
  {
    device_path_ = declare_parameter<std::string>("device", "/dev/input/event3");
    keyboard_fd_ = open(device_path_.c_str(), O_RDONLY | O_NONBLOCK);
    if (keyboard_fd_ < 0) {
      throw std::runtime_error(
              "Cannot open keyboard device " + device_path_ + ": " + std::strerror(errno));
    }

    key_states_.fill(0);
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
    publisher_ = create_publisher<std_msgs::msg::Int32MultiArray>(
      "/nero_keyboard_state", qos);
    timer_ = create_wall_timer(5ms, std::bind(&NeroKeyboardReader::tick, this));

    RCLCPP_INFO(get_logger(), "NERO keyboard device: %s", device_path_.c_str());
    RCLCPP_INFO(get_logger(), "Keys: 1-7 select joint, A/D jog, SPACE home, E E-stop");
  }

  ~NeroKeyboardReader() override
  {
    if (keyboard_fd_ >= 0) {
      close(keyboard_fd_);
    }
  }

private:
  void tick()
  {
    input_event event{};
    while (true) {
      const ssize_t bytes_read = read(keyboard_fd_, &event, sizeof(event));
      if (bytes_read < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK) {
          RCLCPP_ERROR(get_logger(), "Keyboard read failed: %s", std::strerror(errno));
        }
        break;
      }
      if (bytes_read != static_cast<ssize_t>(sizeof(event))) {
        break;
      }
      if (event.type != EV_KEY) {
        continue;
      }
      const int index = key_index(event.code);
      if (index >= 0) {
        key_states_[static_cast<std::size_t>(index)] = event.value == 0 ? 0 : 1;
      }
    }

    std_msgs::msg::Int32MultiArray message;
    message.data.assign(key_states_.begin(), key_states_.end());
    publisher_->publish(message);
  }

  static int key_index(const uint16_t code)
  {
    switch (code) {
      case KEY_1: return 0;
      case KEY_2: return 1;
      case KEY_3: return 2;
      case KEY_4: return 3;
      case KEY_5: return 4;
      case KEY_6: return 5;
      case KEY_7: return 6;
      case KEY_A: return 7;
      case KEY_D: return 8;
      case KEY_SPACE: return 9;
      case KEY_E: return 10;
      default: return -1;
    }
  }

  int keyboard_fd_;
  std::string device_path_;
  std::array<int32_t, KEY_COUNT> key_states_{};
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<NeroKeyboardReader>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("nero_keyboard"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
