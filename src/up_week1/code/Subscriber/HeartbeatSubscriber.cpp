#include "HeartbeatSubscriber.hpp"

HeartbeatSubscriber::HeartbeatSubscriber(void)
	: rclcpp::Node("heartbeat_subscriber")
{
	this->_sub = this->create_subscription<std_msgs::msg::String>(
		"/up/heartbeat", 10,
		std::bind(&HeartbeatSubscriber::_onMessage, this, std::placeholders::_1));
	RCLCPP_INFO(this->get_logger(), "heartbeat_subscriber started");
}

HeartbeatSubscriber::HeartbeatSubscriber(const HeartbeatSubscriber &other)
	: rclcpp::Node("heartbeat_subscriber")
{
	(void)other;
}

HeartbeatSubscriber	&HeartbeatSubscriber::operator=(const HeartbeatSubscriber &other)
{
	(void)other;
	return (*this);
}

HeartbeatSubscriber::~HeartbeatSubscriber(void)
{
}

void	HeartbeatSubscriber::_onMessage(const std_msgs::msg::String::SharedPtr msg)
{
	RCLCPP_INFO(this->get_logger(), "received: %s", msg->data.c_str());
}
