#include "CameraMonitor.hpp"

int	main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<CameraMonitor>());
	rclcpp::shutdown();
	return (0);
}
