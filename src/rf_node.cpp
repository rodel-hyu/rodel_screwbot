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

#define PORT_NAME "/dev/ttyAMA2"
#define PORT_CFLAG  (CS8 | CREAD | B9600)
#define PORT_VTIME  (1)
#define PORT_VMIN (255)

class RfPublisher : public rclcpp::Node
{
  public:
    RfPublisher()
    : Node("rf_publisher")
    {
      publisher_ = this->create_publisher<sensor_msgs::msg::Joy>("rf", 10);
      msg_.axes.resize(2);
      msg_.buttons.resize(1);

      count_ = 0;
      state_ = 0;

      struct termios newtio;

      // Open port
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
    }

    ~RfPublisher(){
      close(fd_);
    }

    void loop()
    {

      fd_set readfds;
      struct timeval timeout;
      int timeOut = 50; // 50ms

      char buff;

      FD_ZERO(&readfds);
      timeout.tv_sec = timeOut / 1000;
      timeout.tv_usec = (timeOut % 1000) * 1000;

      FD_SET(fd_, &readfds);
      select(fd_ + 1, &readfds, NULL, NULL, &timeout);

      if (FD_ISSET(fd_, &readfds)){

        read(fd_, &buff, 1);
        switch(state_){
        case 0 :
          if(buff > 100){
            mode_ = buff;
            state_ = 1;
          }
          break;
        case 1 :
          if(buff > 100){
            state_ = 0;
            break;
          }
          vrx_ = buff;
          state_ = 2;
          break;
        case 2 :
          if(buff > 100){
            state_ = 0;
            break;
          }
          vry_ = buff;
          state_ = 3;
          break;
        }

        if(state_ == 3){ // success
          state_ = 0;

          msg_.header.frame_id = std::to_string(count_++);
          msg_.axes[0] = (float)vrx_ / 100.0f;
          msg_.axes[1] = (float)vry_ / 100.0f;

          int button_prev = msg_.buttons[0];

          switch(mode_){
            case 0x6A: msg_.buttons[0] = 1; break;
            case 0x6B: msg_.buttons[0] = 2; break;
            case 0x6C: msg_.buttons[0] = 3; break;
            case 0x6D: msg_.buttons[0] = 4; break;
            default : msg_.buttons[0] = 0; break;
          }

          if(button_prev != msg_.buttons[0]){
            RCLCPP_INFO(this->get_logger(), "Button Clicked! [%d] -> [%d]", button_prev, msg_.buttons[0]);
          }

          publisher_->publish(msg_);
        }

      }
    }


  private:    
    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr publisher_;
    
    sensor_msgs::msg::Joy msg_;
    uint32_t count_;

    int fd_;
    int state_;

    int mode_;
    int vrx_;
    int vry_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  std::shared_ptr<RfPublisher> pub = std::make_shared<RfPublisher>();

  while(rclcpp::ok()){
    pub->loop();
    rclcpp::spin_some(pub);
  }

  rclcpp::shutdown();
  return 0;
}