#ifndef CAMERAMONITOR_HPP
# define CAMERAMONITOR_HPP

# include <string>
# include "rclcpp/rclcpp.hpp"
# include "sensor_msgs/msg/image.hpp"

class CameraMonitor : public rclcpp::Node
{
	public:
		CameraMonitor(void);
		CameraMonitor(const CameraMonitor &other);
		CameraMonitor	&operator=(const CameraMonitor &other);
		~CameraMonitor(void);

	private:
		void													_onImage(const sensor_msgs::msg::Image::SharedPtr msg);
		void													_onTimer(void);
		rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr	_sub;
		rclcpp::TimerBase::SharedPtr							_timer;
		unsigned long											_count;
		unsigned int											_width;
		unsigned int											_height;
		std::string												_encoding;
};

#endif
