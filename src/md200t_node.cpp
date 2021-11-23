/**********************************************************************
- 홀센서 표시 뒤집혀 있음
- 모터드라이버 UVW(빨흰검) <-> WUV(파노초)
- BAUDRATE 19200

- Init
  1. (17) PID_USE_LIMIT_SW = 0
  2. (21) PID_HALL_TYPE = 10
  3. (235) PID_TQ_GAIN = 10, 0, 20, 0
    * PGain = 10, IGain = 20
  4. (203) PID_GAIN = 10, 0, 144, 1, 244, 1
    * Pos_PGain = 10, Spd_PGain = 400, Spd_IGain = 500
  5. (10) PID_COMMAND = 1, 61
    * CMD_PNT_MAIN_DATA_BC_ON -> (210) PID_PNT_MAIN_DATA 자동응답

 - Control
  * (207) PID_PNT_VEL_CMD => 모터 1,2 속도 입력
  * (174) PID_PNT_TQ_OFF => 모터 1,2 토크 OFF

  * 받을 때 : 0x80 0xB7 로 시작
  * 보낼 때 : 0xB7 0x80 로 시작
**********************************************************************/

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
#include "sensor_msgs/msg/joy.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

#define PORT_NAME "/dev/ttyAMA0"
#define PORT_CFLAG  (CS8 | CREAD | B19200)
#define PORT_VTIME  (1)
#define PORT_VMIN (255)

class Md200TNode : public rclcpp::Node
{
public:
  Md200TNode()
    : Node("md200t_node")
  {
    // Open port
    struct termios newtio;

    fd_ = open(PORT_NAME, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "Fail to open port %s", PORT_NAME);
      return;
    }

    // Open gpio
    fd_gpio_ = open("/sys/class/gpio/export", O_WRONLY);
    write(fd_gpio_, "19", 2);
    close(fd_gpio_);

    // Set gpio to output
    usleep(100000);
    fd_gpio_ = open("/sys/class/gpio/gpio19/direction", O_WRONLY);
    write(fd_gpio_, "out", 3);
    close(fd_gpio_);

    usleep(100000);
    fd_gpio_ = open("/sys/class/gpio/gpio19/value", O_WRONLY);

    set_gpio(0);

    memset(&newtio, 0, sizeof(newtio));
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;
    newtio.c_cflag = PORT_CFLAG;
    newtio.c_lflag = 0;
    newtio.c_cc[VTIME] = PORT_VTIME;
    newtio.c_cc[VMIN] = PORT_VMIN;
    tcflush(fd_, TCIFLUSH);
    tcsetattr(fd_, TCSANOW, &newtio);

    uint8_t tmp[10];
    // [[ Init MD200T ]]

    // 1. (17) PID_USE_LIMIT_SW = 0
    tmp[0] = 0;
    sendFrame(1, 17, 1, tmp);

    // 2. (20) PID_HALL_TYPE = 10
    tmp[0] = 10;
    sendFrame(1, 20, 1, tmp);
    usleep(300000);

    // 3. (235) PID_TQ_GAIN = 10, 0, 20, 0 (PGain = 10, IGan = 20)
    tmp[0] = 10; tmp[1] = 0; tmp[2] = 20; tmp[3] = 0;
    sendFrame(1, 235, 4, tmp);
    usleep(300000);

    // 4. (203) PID_GAIN = 10, 0 ,144, 1, 244, 1 (Pos_PGain = 10, Spd_PGain = 400, Spd_IGain = 500)
    tmp[0] = 10; tmp[1] = 0; tmp[2] = 144; tmp[3] = 1; tmp[4] = 244; tmp[5] = 1;
    sendFrame(1, 203, 6, tmp);
    usleep(300000);

    // 5. (10) PID_COMMAND = 61 (CMD_PNT_MAIN_DATA_BC_ON)
    tmp[0] = 61;
    sendFrame(1, 10, 1, tmp);
    usleep(300000);

    // Flush buffer
    tcflush(fd_, TCIFLUSH);

    motor_output_msg_.buttons.resize(2);
    motor_output_msg_.axes.resize(2);

    publisher_ = this->create_publisher<sensor_msgs::msg::Joy>("md200t/outputs", 10);
    subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
                    "md200t/inputs", 1, std::bind(&Md200TNode::sub_callback, this, _1));

    timer_ = this->create_wall_timer(
               100ms, std::bind(&Md200TNode::timer_callback, this)); // 10Hz
  }

  ~Md200TNode() {
    uint8_t tmp[3];

    // (10) PID_COMMAND = 62 (CMD_PNT_MAIN_BC_OFF)
    tmp[0]  = 62;
    sendFrame(1, 10, 1, tmp);

    usleep(300000);

    // (174) PID_PNT_TQ_OFF
    tmp[0] = 1;
    tmp[1] = 1;
    tmp[2] = 0;
    sendFrame(1, 174, 3, tmp);

    // Unexport gpio
    set_gpio(0);
    close(fd_gpio_);
    fd_gpio_ = open("/sys/class/gpio/unexport", O_WRONLY);
    write(fd_gpio_, "19", 2);

    close(fd_gpio_);
    close(fd_);
  }


private:

  void timer_callback()
  {
    // Read (193) PID_MAIN_DATA
    struct timeval timeout;
    fd_set set;
    int rv;

    FD_ZERO(&set);
    FD_SET(fd_, &set);

    timeout.tv_sec = 0;
    timeout.tv_usec = 5000;

    rv = select(fd_ + 1, &set, NULL, NULL, &timeout);
    if (rv > 0) {
      rv = read(fd_, &rxBuffer_, 64);
      if(rv < 24) return;
      if(rxBuffer_[0] != 0x80) return;
      if(rxBuffer_[1] != 0xB7) return;
      if(rxBuffer_[2] != 0x01) return;
      if(rxBuffer_[3] != 210) return;
      if(rxBuffer_[4] != 18) return;

      int16_t tmp16;
      
      tmp16 = (uint16_t)rxBuffer_[5] | ((uint16_t)rxBuffer_[6] << 8);
      motor_output_msg_.axes[0] = tmp16;

      tmp16 = (uint16_t)rxBuffer_[14] | ((uint16_t)rxBuffer_[15] << 8);
      motor_output_msg_.axes[1] = tmp16;

      publisher_->publish(motor_output_msg_);
    }
  }

  void sub_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    // (207) PID_PNT_VEL_CMD
    uint8_t tmp[7];
    int16_t tmp16;

    tmp[0] = msg->buttons[0] == 1 ? 1 : 0;

    tmp16 = (int16_t)msg->axes[0];
    tmp[1] = (uint8_t)tmp16;
    tmp[2] = (uint8_t)(tmp16 >> 8);

    tmp[3] = msg->buttons[1] == 1 ? 1 : 0;

    tmp16 = (int16_t)msg->axes[1];
    tmp[4] = (uint8_t)tmp16;
    tmp[5] = (uint8_t)(tmp16 >> 8);

    tmp[6] = 0;

    sendFrame(1, 207, 7, tmp);

    // (174) PID_PNT_TQ_OFF
    if(msg->buttons[0] == 0 || msg->buttons[1] == 0){
      tmp[0] = msg->buttons[0] == 1 ? 0 : 1;
      tmp[1] = msg->buttons[1] == 1 ? 0 : 1;
      tmp[2] = 0;
      sendFrame(1, 174, 3, tmp);
    }
  }

  void set_gpio(int val) {
    if (val == 1) {
      lseek(fd_gpio_, 0, SEEK_SET);
      write(fd_gpio_, "1", 1);
      tcdrain(fd_gpio_);
    } else {
      lseek(fd_gpio_, 0, SEEK_SET);
      write(fd_gpio_, "0", 1);
      tcdrain(fd_gpio_);
    }
  }

  void sendFrame(uint8_t id, uint8_t pid, uint8_t dlen, uint8_t* data) {
    txBuffer_[0] = 0xB7;
    txBuffer_[1] = 0x80;
    txBuffer_[2] = id;
    txBuffer_[3] = pid;
    txBuffer_[4] = dlen;
    memcpy(txBuffer_ + 5, data, dlen);

    // Checksum
    uint8_t cksum = 0;
    int i;
    for (i = 0; i < 5 + dlen; i++) {
      cksum += txBuffer_[i];
    }

    txBuffer_[5 + dlen] = -cksum;

    set_gpio(1);
    write(fd_, txBuffer_, 6 + dlen);
    tcdrain(fd_);
    set_gpio(0);
  }

  // ROS variables
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscriber_;

  sensor_msgs::msg::Joy motor_output_msg_;

  int fd_;
  char txBuffer_[64];
  char rxBuffer_[64];

  int fd_gpio_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Md200TNode>());

  rclcpp::shutdown();
  return 0;
}