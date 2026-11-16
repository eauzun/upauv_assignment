#include "CameraMonitor.hpp"

CameraMonitor::CameraMonitor(void)
	: rclcpp::Node("camera_monitor"), _count(0), _width(0), _height(0)
{
	this->_sub = this->create_subscription<sensor_msgs::msg::Image>(
		"/camera/image_raw", 10,
		std::bind(&CameraMonitor::_onImage, this, std::placeholders::_1));
	this->_timer = this->create_wall_timer(
		std::chrono::seconds(1),
		std::bind(&CameraMonitor::_onTimer, this));
	RCLCPP_INFO(this->get_logger(), "camera_monitor started");
}

CameraMonitor::CameraMonitor(const CameraMonitor &other)
	: rclcpp::Node("camera_monitor"), _count(other._count),
	_width(other._width), _height(other._height), _encoding(other._encoding)
{
}

CameraMonitor	&CameraMonitor::operator=(const CameraMonitor &other)
{
	if (this != &other)
	{
		this->_count = other._count;
		this->_width = other._width;
		this->_height = other._height;
		this->_encoding = other._encoding;
	}
	return (*this);
}

CameraMonitor::~CameraMonitor(void)
{
}

void	CameraMonitor::_onImage(const sensor_msgs::msg::Image::SharedPtr msg)
{
	this->_count++;
	this->_width = msg->width;
	this->_height = msg->height;
	this->_encoding = msg->encoding;
}

void	CameraMonitor::_onTimer(void)
{
	RCLCPP_INFO(this->get_logger(), "width=%u height=%u encoding=%s total=%lu",
		this->_width, this->_height, this->_encoding.c_str(), this->_count);
}
