#include <array>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <functional>
#include <stdexcept>
#include <string>

#include <linux/input.h>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>

using namespace std::chrono_literals;

class KeyboardReader : public rclcpp::Node
{
public:
    static constexpr std::size_t KEY_COUNT = 12;

    enum KeyIndex : std::size_t
    {
        INDEX_W = 0,
        INDEX_S,
        INDEX_A,
        INDEX_D,
        INDEX_SPACE,
        INDEX_UP,
        INDEX_DOWN,
        INDEX_LEFT,
        INDEX_RIGHT,
        INDEX_PAGEUP,
        INDEX_PAGEDOWN,
        INDEX_RESET_CORRECTION
    };

    KeyboardReader()
    : Node("keyboard_reader"),
      keyboard_fd_(-1)
    {
        device_path_ = this->declare_parameter<std::string>(
            "device",
            "/dev/input/event3"
        );

        keyboard_fd_ = open(
            device_path_.c_str(),
            O_RDONLY | O_NONBLOCK
        );

        if (keyboard_fd_ < 0) {
            throw std::runtime_error(
                "Cannot open keyboard device: " +
                device_path_ +
                ", error: " +
                std::strerror(errno)
            );
        }

        key_states_.fill(0);

        auto qos = rclcpp::QoS(
            rclcpp::KeepLast(1)
        );

        // 1000 Hz 数据不需要积压
        qos.best_effort();

        publisher_ =
            this->create_publisher<
                std_msgs::msg::Int32MultiArray
            >(
                "/keyboard_state",
                qos
            );

        // 1 ms周期，理论发布频率1000 Hz
        timer_ = this->create_wall_timer(
            1ms,
            std::bind(
                &KeyboardReader::timerCallback,
                this
            )
        );

        RCLCPP_INFO(
            this->get_logger(),
            "Keyboard device: %s",
            device_path_.c_str()
        );

        RCLCPP_INFO(
            this->get_logger(),
            "Publishing /keyboard_state at 1000 Hz"
        );

        RCLCPP_INFO(
            this->get_logger(),
            "Data order: "
            "[W, S, A, D, SPACE, UP, DOWN, LEFT, RIGHT, PAGEUP, PAGEDOWN, _]"
        );
    }

    ~KeyboardReader() override
    {
        if (keyboard_fd_ >= 0) {
            close(keyboard_fd_);
            keyboard_fd_ = -1;
        }
    }

private:
    void timerCallback()
    {
        readKeyboardEvents();
        publishKeyboardState();
    }

    void readKeyboardEvents()
    {
        input_event event {};

        while (true) {
            const ssize_t bytes_read = read(
                keyboard_fd_,
                &event,
                sizeof(event)
            );

            if (bytes_read < 0) {
                if (
                    errno == EAGAIN ||
                    errno == EWOULDBLOCK
                ) {
                    break;
                }

                RCLCPP_ERROR(
                    this->get_logger(),
                    "Keyboard read error: %s",
                    std::strerror(errno)
                );

                break;
            }

            if (
                bytes_read !=
                static_cast<ssize_t>(sizeof(event))
            ) {
                break;
            }

            if (event.type != EV_KEY) {
                continue;
            }

            processKeyEvent(
                event.code,
                event.value
            );
        }
    }

    void processKeyEvent(
        const uint16_t key_code,
        const int32_t value)
    {
        const int index = keyCodeToIndex(key_code);

        if (index < 0) {
            return;
        }

        /*
         * Linux EV_KEY:
         *
         * value = 0：松开
         * value = 1：按下
         * value = 2：自动重复
         */

        if (value == 1) {
            key_states_[index] = 1;

            RCLCPP_INFO(
                this->get_logger(),
                "%s PRESS",
                keyName(index)
            );
        }
        else if (value == 0) {
            key_states_[index] = 0;

            RCLCPP_INFO(
                this->get_logger(),
                "%s RELEASE",
                keyName(index)
            );
        }
        else if (value == 2) {
            // 长按重复事件
            // 状态已经是1，不需要改变
            key_states_[index] = 1;
        }
    }

    void publishKeyboardState()
    {
        std_msgs::msg::Int32MultiArray message;

        message.data.assign(
            key_states_.begin(),
            key_states_.end()
        );

        publisher_->publish(message);
    }

    int keyCodeToIndex(
        const uint16_t key_code) const
    {
        switch (key_code) {
            case KEY_W:
                return INDEX_W;

            case KEY_S:
                return INDEX_S;

            case KEY_A:
                return INDEX_A;

            case KEY_D:
                return INDEX_D;

            case KEY_SPACE:
                return INDEX_SPACE;

            case KEY_UP:
                return INDEX_UP;

            case KEY_DOWN:
                return INDEX_DOWN;

            case KEY_LEFT:
                return INDEX_LEFT;

            case KEY_RIGHT:
                return INDEX_RIGHT;

            case KEY_PAGEUP:
                return INDEX_PAGEUP;

            case KEY_PAGEDOWN:
                return INDEX_PAGEDOWN;

            case KEY_MINUS:
                return INDEX_RESET_CORRECTION;

            default:
                return -1;
        }
    }

    const char* keyName(
        const int index) const
    {
        static constexpr std::array<
            const char*,
            KEY_COUNT
        > names {
            "W",
            "S",
            "A",
            "D",
            "SPACE",
            "UP",
            "DOWN",
            "LEFT",
            "RIGHT",
            "PAGEUP",
            "PAGEDOWN",
            "_"
        };

        if (
            index < 0 ||
            index >= static_cast<int>(KEY_COUNT)
        ) {
            return "UNKNOWN";
        }

        return names[index];
    }

    int keyboard_fd_;

    std::string device_path_;

    std::array<
        int32_t,
        KEY_COUNT
    > key_states_;

    rclcpp::Publisher<
        std_msgs::msg::Int32MultiArray
    >::SharedPtr publisher_;

    rclcpp::TimerBase::SharedPtr timer_;
};


int main(
    int argc,
    char** argv)
{
    rclcpp::init(argc, argv);

    try {
        auto node =
            std::make_shared<KeyboardReader>();

        rclcpp::spin(node);
    }
    catch (const std::exception& exception) {
        RCLCPP_FATAL(
            rclcpp::get_logger("keyboard_reader"),
            "%s",
            exception.what()
        );
    }

    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }

    return 0;
}
