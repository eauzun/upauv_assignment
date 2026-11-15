#ifndef HEARTBEATPUBLISHER_HPP
# define HEARTBEATPUBLISHER_HPP

# include <string>
# include "rclcpp/rclcpp.hpp"
# include "std_msgs/msg/string.hpp"

class HeartbeatPublisher : public rclcpp::Node
{
	public:
		HeartbeatPublisher(void);
		HeartbeatPublisher(const HeartbeatPublisher &other);
		HeartbeatPublisher	&operator=(const HeartbeatPublisher &other);
		~HeartbeatPublisher(void);

	private:
		void												_onTimer(void);
		rclcpp::Publisher<std_msgs::msg::String>::SharedPtr	_pub;
		rclcpp::TimerBase::SharedPtr						_timer;
		unsigned long										_seq;
};

#endif
