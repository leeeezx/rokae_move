#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/string.hpp>
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <cstring>
#include <stdexcept>
#include <vector>
#include <iostream>
#include <sstream>

class PipeReceiver : public rclcpp::Node {
public:
    PipeReceiver(const std::string& pipePath)
        : Node("pipe_receiver"), pipePath_(pipePath), fd_(-1) {
        // 创建定时器，定时读取管道数据
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&PipeReceiver::timer_callback, this)
        );
    }

    ~PipeReceiver() {
        close_pipe();
    }

    void open_pipe() {
        if (!std::filesystem::exists(pipePath_)) {
            if (mkfifo(pipePath_.c_str(), 0666) == -1) {
                throw std::runtime_error("创建管道失败: " + std::string(strerror(errno)));
            }
            RCLCPP_INFO(this->get_logger(), "创建命名管道: %s", pipePath_.c_str());
        }
        fd_ = ::open(pipePath_.c_str(), O_RDONLY | O_NONBLOCK);
        if (fd_ == -1) {
            throw std::runtime_error("打开管道失败: " + std::string(strerror(errno)));
        }
        RCLCPP_INFO(this->get_logger(), "管道已打开: %s", pipePath_.c_str());
    }

    void close_pipe() {
        if (fd_ != -1) {
            ::close(fd_);
            fd_ = -1;
            RCLCPP_INFO(this->get_logger(), "管道已关闭");
        }
    }

private:
    std::string pipePath_;
    int fd_;
    rclcpp::TimerBase::SharedPtr timer_;

    void timer_callback() {
        if (fd_ == -1) {
            try {
                open_pipe();
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "打开管道错误: %s", e.what());
                return;
            }
        }

        // 读取数据长度
        uint32_t dataSize;
        ssize_t bytesRead = read(fd_, &dataSize, sizeof(dataSize));
        if (bytesRead == 0) {
            // 没有写入者
            close_pipe();
            return;
        }
        if (bytesRead != sizeof(dataSize)) {
            if (errno != EAGAIN && errno != EWOULDBLOCK) {
                RCLCPP_ERROR(this->get_logger(), "读取数据长度错误: %s", strerror(errno));
            }
            return;
        }

        // 读取实际数据
        std::vector<char> buffer(dataSize);
        bytesRead = read(fd_, buffer.data(), dataSize);
        if (bytesRead != static_cast<ssize_t>(dataSize)) {
            RCLCPP_ERROR(this->get_logger(), "读取数据内容错误: %s", strerror(errno));
            return;
        }

        std::string data(buffer.begin(), buffer.end());
        RCLCPP_INFO(this->get_logger(), "接收到数据长度: %u, 内容: %s", dataSize, data.c_str());
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<PipeReceiver>("/tmp/sensor_data_pipe");
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        std::cerr << "程序异常终止: " << e.what() << std::endl;
    }
    rclcpp::shutdown();
    return 0;
}
