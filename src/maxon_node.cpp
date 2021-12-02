#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <unistd.h>
#include <pthread.h>
#include <net/if.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/joy.hpp"

#include "CANOpen.h"

int s_can; // Global socketCAN file descriptor
pthread_mutex_t mutex_can; // Global CANOpen mutex

using namespace std::chrono_literals;
using std::placeholders::_1;

/********************************************************************/
// CAN Thread functions
/********************************************************************/
void* canopen_rxloop(void* d) {

	int *activate = (int*)d;

	struct can_frame frame;

	struct timeval timeout;
	fd_set set;
	int rv;
	while (*activate) {
		FD_ZERO(&set);
		FD_SET(s_can, &set);

		timeout.tv_sec = 0;
		timeout.tv_usec = 5000;

		rv = select(s_can + 1, &set, NULL, NULL, &timeout);
		if (rv > 0) {
			read(s_can, &frame, sizeof(frame));
			CANOpen_addRxBuffer(frame.can_id, frame.data);
		} else {
			CANOpen_timerLoop();
		}
	}

	pthread_exit(NULL);

}

/********************************************************************/
// Maxon Motor Class
/********************************************************************/
class MaxonMotor
{
public:
	static const int8_t CONTROL_PPM = 1;
	static const int8_t CONTROL_PVM = 3;

	MaxonMotor(uint8_t id, int8_t control_type) {
		id_ = id;
		control_type_ = control_type;
	}

	~MaxonMotor() {
		stop();
	}

	void init() {
		// Mapping to RPDO3
		CANOpen_mappingPDO_init(&RPDO_);
		CANOpen_mappingPDO_uint16(&RPDO_, &controlword_);
		CANOpen_mappingPDO_int32(&RPDO_, &target_value_);

		// Mapping to TPDO3 or TPDO4
		CANOpen_mappingPDO_init(&TPDO_);
		CANOpen_mappingPDO_uint16(&TPDO_, &statusword_);
		CANOpen_mappingPDO_int32(&TPDO_, &actual_value_);

		switch (control_type_) {
		case CONTROL_PPM:
			tpdo_ch_ = 3;
			CANOpen_writeOD_int8(id_, 0x6060, 0x00, 1, 100); // Set Profile Position Mode
			CANOpen_writeOD_uint32(id_, 0x1800, 0x01, 0xC0000180 | id_, 100); // Disable Node1 TPDO1
			CANOpen_writeOD_uint8(id_, 0x1802, 0x02, 0xFD, 100); // Set Node1 TPDO3 to async RTR only mode
			CANOpen_writeOD_uint32(id_, 0x1802, 0x01, 0x380UL | id_, 100); // Enable Node1 TPDO3 with RTR
			break;

		case CONTROL_PVM:
			tpdo_ch_ = 4;
			CANOpen_writeOD_int8(id_, 0x6060, 0x00, 1, 100); // Set Profile Position Mode
			CANOpen_writeOD_uint32(id_, 0x1800, 0x01, 0xC0000180 | id_, 100); // Disable Node1 TPDO1
			CANOpen_writeOD_uint8(id_, 0x1803, 0x02, 0xFD, 100); // Set Node1 TPDO4 to async RTR only mode
			CANOpen_writeOD_uint32(id_, 0x1803, 0x01, 0x480UL | id_, 100); // Enable Node1 TPDO3 with RTR
			break;
		}
	}

	void start() {
		uint8_t d[8];
		uint8_t len;

		do {
			CANOpen_writeOD_int8(id_, 0x6060, 0x00, control_type_, 100);
			CANOpen_readOD(id_, 0x6061, 0x00, d, &len, 100);
		} while ((int8_t)d[0] != control_type_);


		CANOpen_writeOD_uint16(id_, 0x6040, 0x00, 0x0006, 100);
		usleep(1000);
		CANOpen_writeOD_uint16(id_, 0x6040, 0x00, 0x000F, 100);

	}

	void stop() {
		controlword_ = 0;
		CANOpen_sendPDO(id_, tpdo_ch_, &RPDO_);
	}

	bool read(uint16_t *statusword, int32_t *actual_value) {
		CANOpen_sendTpdoRTR(id_, tpdo_ch_);
		if (CANOpen_readPDO(id_, tpdo_ch_, &TPDO_, 20) != CO_OK) {
			return false;
		} else {
			*actual_value = actual_value_;
			*statusword = statusword_;
			return true;
		}
	}

	void new_setpoint(int32_t target_value) {
		switch (control_type_) {
		case CONTROL_PPM:
			// Toggle "New Position"
			controlword_ = 0x0F;
			target_value_ = target_value;
			CANOpen_sendPDO(id_, tpdo_ch_, &RPDO_);
			break;

		case CONTROL_PVM:
			controlword_ = 0x0F;
			target_value_ = target_value;
			break;
		}
	}

	void go_setpoint() {
		switch (control_type_) {
		case CONTROL_PPM:
			// Write desired value & set immediately
			controlword_ = 0x7F;
			CANOpen_sendPDO(id_, tpdo_ch_, &RPDO_);
			break;

		case CONTROL_PVM:
			controlword_ = 0x0F;
			CANOpen_sendPDO(id_, tpdo_ch_, &RPDO_);
			break;
		}
	}

	int id() {
		return id_;
	}

private:
	uint8_t id_;
	uint8_t control_type_;
	uint8_t tpdo_ch_;

	CO_PDOStruct TPDO_;
	CO_PDOStruct RPDO_;

	uint16_t controlword_;
	uint16_t statusword_;

	int32_t target_value_;
	int32_t actual_value_;
};

/********************************************************************/
// ROS Node
/********************************************************************/
class MaxonNode : public rclcpp::Node
{
public:
	MaxonNode() : Node("maxon_node")
	{
		/**********************************************************************/
		// Enable SocketCAN
		/**********************************************************************/
		// socketCAN variables
		int ret;
		struct sockaddr_can addr;
		struct ifreq ifr;
		struct can_filter rfilter[1];

		// Get socketCAN ----------------------------------------------------->
		// [[ 1.Create socket ]]
		s_can = socket(PF_CAN, SOCK_RAW, CAN_RAW);
		if (s_can < 0) {
			RCLCPP_ERROR(this->get_logger(), "socketCAN PF_CAN failed.");
			return;
		}

		// [[ 2.Specify can0 device ]]
		strcpy(ifr.ifr_name, "can0");
		ret = ioctl(s_can, SIOCGIFINDEX, &ifr);
		if (ret < 0) {
			RCLCPP_ERROR(this->get_logger(), "socketCAN ioctl failed.");
			return;
		}

		// [[ 3.Bind the socket to can0 ]]
		addr.can_family = AF_CAN;
		addr.can_ifindex = ifr.ifr_ifindex;
		ret = bind(s_can, (struct sockaddr *)&addr, sizeof(addr));
		if (ret < 0) {
			RCLCPP_ERROR(this->get_logger(), "socketCAN bind failed.");
			return;
		}

		// [[ 4.Receive all frame ]]
		rfilter[0].can_id = 0x000;
		rfilter[0].can_mask = 0x000;
		setsockopt(s_can, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));

		RCLCPP_INFO(this->get_logger(), "can0 socketCAN Enabled.");
		// Get socketCAN <-----------------------------------------------------

		activate_ = 1;

		// Create CAN related threads ---------------------------------------->
		pthread_mutex_init(&mutex_can, NULL);

		if (pthread_create(&p_thread_, NULL, canopen_rxloop, &activate_) < 0) {
			RCLCPP_ERROR(this->get_logger(), "can0 RX loop thread create error.");
			return;
		}

		RCLCPP_INFO(this->get_logger(), "can0 Threads established.");
		// Create CAN related threads <----------------------------------------

		/**********************************************************************/
		// Setting CANOpen
		/**********************************************************************/
		CANOpen_init();

		// Reset all node
		CANOpen_NMT(CO_RESET, 0);

		sleep(2);

		m_[0] = new MaxonMotor(0x01, MaxonMotor::CONTROL_PPM);
		m_[1] = new MaxonMotor(0x02, MaxonMotor::CONTROL_PVM);
		m_[2] = new MaxonMotor(0x03, MaxonMotor::CONTROL_PPM);
		m_[3] = new MaxonMotor(0x04, MaxonMotor::CONTROL_PVM);

		m_[0]->init();
		m_[1]->init();
		m_[2]->init();
		m_[3]->init();

		m_[0]->start();
		m_[1]->start();
		m_[2]->start();
		m_[3]->start();

		CANOpen_NMT(CO_OP, 0);

		/**********************************************************************/
		// Setting ROS Publisher & Subscribers
		/**********************************************************************/
		//subscriber_ = std::make_shared<CAN0Subscriber>();

		publisher_ = this->create_publisher<sensor_msgs::msg::Joy>("maxon/outputs", 10);
		subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
		                  "maxon/inputs", 1, std::bind(&MaxonNode::sub_callback, this, _1));

		motor_output_msg_.axes.resize(4);
		motor_output_msg_.buttons.resize(4);

		timer_ = this->create_wall_timer(25ms, std::bind(&MaxonNode::timer_callback, this));
	}

	~MaxonNode() {
		int status;

		CANOpen_NMT(CO_RESET, 0);

		activate_ = 0;
		pthread_join(p_thread_, (void **)&status);
		pthread_mutex_destroy(&mutex_can);
		close(s_can);
	}

private:
	////////////////////////////////////////////////////////////////////////////
	// Main functions
	////////////////////////////////////////////////////////////////////////////
	void timer_callback()
	{
		uint16_t statusword_tmp;
		int32_t actual_value_tmp;

		usleep(1000); // 1ms delay between commands

		int i;
		for(i = 0; i < 4; i++){
			if(m_[i]->read(&statusword_tmp, &actual_value_tmp)){
				motor_output_msg_.buttons[i] = statusword_tmp;
				motor_output_msg_.axes[i] = actual_value_tmp;
			}else{
				RCLCPP_ERROR(this->get_logger(), "Maxon Node%d TPDO Timeout", m_[i]->id());
			}
		}

		publisher_->publish(motor_output_msg_);

	}
	
	void sub_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
		if (msg->buttons[0] == 1) m_[0]->new_setpoint((int32_t)msg->axes[0]);
		if (msg->buttons[1] == 1) m_[1]->new_setpoint((int32_t)msg->axes[1]);
		if (msg->buttons[2] == 1) m_[2]->new_setpoint((int32_t)msg->axes[2]);
		if (msg->buttons[3] == 1) m_[3]->new_setpoint((int32_t)msg->axes[3]);

		usleep(1000); // 1ms delay between commands

		if (msg->buttons[0] == 1) m_[0]->go_setpoint();
		if (msg->buttons[1] == 1) m_[1]->go_setpoint();
		if (msg->buttons[2] == 1) m_[2]->go_setpoint();
		if (msg->buttons[3] == 1) m_[3]->go_setpoint();
	}

	// ROS variables
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr publisher_;
	rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscriber_;

	sensor_msgs::msg::Joy motor_output_msg_;

	// CANOpen variables
	int activate_;
	pthread_t p_thread_;

	// Maxon Node
	MaxonMotor *m_[4];

};


int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<MaxonNode>());
	rclcpp::shutdown();
	return 0;
}