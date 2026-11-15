#include "HeartbeatPublisher.hpp"

HeartbeatPublisher::HeartbeatPublisher(void)
	: rclcpp::Node("heartbeat_publisher"), _seq(0)
{
	RCLCPP_INFO(this->get_logger(), "heartbeat_publisher started");
}

HeartbeatPublisher::HeartbeatPublisher(const HeartbeatPublisher &other)
	: rclcpp::Node("heartbeat_publisher"), _seq(other._seq)
{
}

HeartbeatPublisher	&HeartbeatPublisher::operator=(const HeartbeatPublisher &other)
{
	if (this != &other)
		this->_seq = other._seq;
	return (*this);
}

HeartbeatPublisher::~HeartbeatPublisher(void)
{
}
