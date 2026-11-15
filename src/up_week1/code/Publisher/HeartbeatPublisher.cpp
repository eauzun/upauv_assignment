#include "HeartbeatPublisher.hpp"

HeartbeatPublisher::HeartbeatPublisher(void)
	: rclcpp::Node("heartbeat_publisher"), _seq(0)
{
	this->_timer = this->create_wall_timer(
		std::chrono::seconds(1),
		std::bind(&HeartbeatPublisher::_onTimer, this));
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

void	HeartbeatPublisher::_onTimer(void)
{
	RCLCPP_INFO(this->get_logger(), "tick %lu", this->_seq);
	this->_seq++;
}
