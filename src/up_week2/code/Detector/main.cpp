#include "RedTargetDetector.hpp"

int	main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	try
	{
		rclcpp::spin(std::make_shared<RedTargetDetector>());
	}
	catch (const std::exception &e)
	{
		RCLCPP_FATAL(rclcpp::get_logger("red_target_detector"), "%s", e.what());
		rclcpp::shutdown();
		return (1);
	}
	rclcpp::shutdown();
	return (0);
}
