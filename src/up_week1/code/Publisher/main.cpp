#include "HeartbeatPublisher.hpp"

int	main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<HeartbeatPublisher>());
	rclcpp::shutdown();
	return (0);
}
