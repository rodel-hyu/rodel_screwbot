#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <pthread.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/byte.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joy.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using namespace std::chrono_literals;
using std::placeholders::_1;

#define PORT_NAME "/dev/ttyAMA1"
#define PORT_CFLAG  (CS8 | CREAD | B115200)
#define PORT_VTIME  (1)
#define PORT_VMIN (255)

class DisplayNode : public rclcpp::Node
{
public:
  DisplayNode()
    : Node("display_node")
  {
    publisher_ = this->create_publisher<std_msgs::msg::Byte>("display", 10);

    // Subscriber
    rf_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
                       "rf", 1, std::bind(&DisplayNode::rf_callback, this, _1));
    imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
                        "imu", 1, std::bind(&DisplayNode::imu_callback, this, _1));
    maxon_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
                          "maxon/outputs", 1, std::bind(&DisplayNode::maxon_callback, this, _1));

    // Open port
    struct termios newtio;

    fd_ = open(PORT_NAME, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "Fail to open port %s", PORT_NAME);
      return;
    }

    memset(&newtio, 0, sizeof(newtio));
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;
    newtio.c_cflag = PORT_CFLAG;
    newtio.c_lflag = 0;
    newtio.c_cc[VTIME] = PORT_VTIME;
    newtio.c_cc[VMIN] = PORT_VMIN;
    tcflush(fd_, TCIFLUSH);
    tcsetattr(fd_, TCSANOW, &newtio);

    // 115200bps / 8 = 14400Bytes/s
    timer_ = this->create_wall_timer(
               100ms, std::bind(&DisplayNode::timer_callback, this)); // 10Hz
  }

  ~DisplayNode() {
    close(fd_);
  }


private:

  void timer_callback()
  {
    // Write data to display
    int nwrite;

    if (rf_msg_ != nullptr) {
      nwrite = sprintf(txBuffer_, "j0.val=%d\xff\xff\xff", (int)(rf_msg_->axes[0] * 100));
      nwrite += sprintf(txBuffer_ + nwrite, "j1.val=%d\xff\xff\xff", (int)(rf_msg_->axes[1] * 100));
      nwrite += sprintf(txBuffer_ + nwrite, "b6.bco=%d\xff\xff\xff", rf_msg_->buttons[0] == 1 ? 64528 : 65535);
      nwrite += sprintf(txBuffer_ + nwrite, "b7.bco=%d\xff\xff\xff", rf_msg_->buttons[0] == 2 ? 64528 : 65535);
      nwrite += sprintf(txBuffer_ + nwrite, "b8.bco=%d\xff\xff\xff", rf_msg_->buttons[0] == 3 ? 64528 : 65535);
      nwrite += sprintf(txBuffer_ + nwrite, "b9.bco=%d\xff\xff\xff", rf_msg_->buttons[0] == 4 ? 64528 : 65535);
      write(fd_, txBuffer_, nwrite);
    }


    if (imu_msg_ != nullptr) {
      // Convert quaternion to euler
      tf2::Quaternion q(
        imu_msg_->orientation.x,
        imu_msg_->orientation.y,
        imu_msg_->orientation.z,
        imu_msg_->orientation.w
      );
      tf2::Matrix3x3 m(q);
      tf2Scalar roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);

      nwrite = sprintf(txBuffer_, "j2.val=%d\xff\xff\xff", (int)(roll * 15.91549431 + 50));
      nwrite += sprintf(txBuffer_ + nwrite, "j3.val=%d\xff\xff\xff", (int)(pitch * 15.91549431 + 50));
      nwrite += sprintf(txBuffer_ + nwrite, "j4.val=%d\xff\xff\xff", (int)(yaw * 15.91549431 + 50));
      write(fd_, txBuffer_, nwrite);
    }

    if (maxon_msg_ != nullptr){
      int i;
      for (i = 0; i < 3; i++) {
        int bco;
        if (maxon_msg_->buttons[i] & 0x08) bco = 64528; // Fault (read)
        else if (maxon_msg_->buttons[i] & 0x80) bco = 65504; // Warning (yellow)
        else if ((maxon_msg_->buttons[i] & 0x400) == 0) bco = 1055; // Target not reached (blue)
        else if ((maxon_msg_->buttons[i] & 0x17) == 0x17) bco = 2016; // Active (green)

        nwrite = sprintf(txBuffer_, "b%d.bco=%d\xff\xff\xff", 10 + i, bco);
        nwrite += sprintf(txBuffer_ + nwrite, "t%d.txt=\"%d %s\"\xff\xff\xff",
          i,
          (int)maxon_msg_->axes[i],
          i%2 == 0 ? "inc" : "inc/s");
        write(fd_, txBuffer_, nwrite);
      }
    }

    // Read display button event
    char buff[5];
    int nread;

    fd_set readfds;
    struct timeval timeout;
    int timeOut = 5; // 5ms

    FD_ZERO(&readfds);
    timeout.tv_sec = timeOut / 1000;
    timeout.tv_usec = (timeOut % 1000) * 1000;

    FD_SET(fd_, &readfds);
    select(fd_ + 1, &readfds, NULL, NULL, &timeout);

    if (FD_ISSET(fd_, &readfds)) {
      nread = read(fd_, buff, 5);

      if (nread >= 5) {
        if (buff[0] != 'p') return;
        if (buff[2] != 0xFF) return;

        if (buff[1] >= '0' && buff[1] <= '9') {
          auto msg = std_msgs::msg::Byte();
          msg.data = buff[1] - '0';
          publisher_->publish(msg);
        }
      }
    }

  }


  void rf_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    rf_msg_ = msg;
  }

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    imu_msg_ = msg;
  }

  void maxon_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    maxon_msg_ = msg;
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Byte>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr rf_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr maxon_subscriber_;

  sensor_msgs::msg::Joy::SharedPtr rf_msg_;
  sensor_msgs::msg::Imu::SharedPtr imu_msg_;
  sensor_msgs::msg::Joy::SharedPtr maxon_msg_;

  int fd_;
  char txBuffer_[256];
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DisplayNode>());

  rclcpp::shutdown();
  return 0;
}