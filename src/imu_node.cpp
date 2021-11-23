#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "Adafruit_Sensor.h"
#include "Adafruit_BNO055.h"
#include "utility/imumaths.h"

using namespace std::chrono_literals;

class IMUPublisher : public rclcpp::Node
{
  public:
    IMUPublisher()
    : Node("minimal_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);

      bno_ = new Adafruit_BNO055(55, 0x28, "/dev/i2c-1");
      if(!bno_->begin()){
        RCLCPP_ERROR(this->get_logger(), "Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
        return;
      }

      /* Use external crystal for better accuracy */
      bno_->setExtCrystalUse(true);

      timer_ = this->create_wall_timer(
      10ms, std::bind(&IMUPublisher::timer_callback, this)); // 100Hz
    }

  private:
    void timer_callback()
    {
      sensors_event_t angVelocityData , linearAccelData;

      bno_->getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
      bno_->getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);

      imu::Vector<3> vec;

      // Get Angular velocity
      data_.angular_velocity.x = angVelocityData.gyro.x;
      data_.angular_velocity.y = angVelocityData.gyro.y;
      data_.angular_velocity.z = angVelocityData.gyro.z;

      // Get Liner accel
      data_.linear_acceleration.x = linearAccelData.acceleration.x;
      data_.linear_acceleration.y = linearAccelData.acceleration.y;
      data_.linear_acceleration.z = linearAccelData.acceleration.z;

      // Get Quaternion
      imu::Quaternion quat = bno_->getQuat();
      data_.orientation.x = quat.x();
      data_.orientation.y = quat.y();
      data_.orientation.z = quat.z();
      data_.orientation.w = quat.w();

      publisher_->publish(data_);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
    size_t count_;

    Adafruit_BNO055 *bno_;
    sensor_msgs::msg::Imu data_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IMUPublisher>());
  rclcpp::shutdown();
  return 0;
}