#include "TargetMover.hpp"

int	main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	try
	{
		rclcpp::spin(std::make_shared<TargetMover>());
	}
	catch (const std::exception &e)
	{
		RCLCPP_FATAL(rclcpp::get_logger("target_mover"), "%s", e.what());
		rclcpp::shutdown();
		return (1);
	}
	rclcpp::shutdown();
	return (0);
}
