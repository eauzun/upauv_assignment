#ifndef HEARTBEATSUBSCRIBER_HPP
# define HEARTBEATSUBSCRIBER_HPP

# include <string>
# include "rclcpp/rclcpp.hpp"
# include "std_msgs/msg/string.hpp"

class HeartbeatSubscriber : public rclcpp::Node
{
	public:
		HeartbeatSubscriber(void);
		HeartbeatSubscriber(const HeartbeatSubscriber &other);
		HeartbeatSubscriber	&operator=(const HeartbeatSubscriber &other);
		~HeartbeatSubscriber(void);

	private:
		void																_onMessage(const std_msgs::msg::String::SharedPtr msg);
		rclcpp::Subscription<std_msgs::msg::String>::SharedPtr				_sub;
};

#endif
