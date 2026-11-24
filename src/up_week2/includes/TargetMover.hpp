#ifndef TARGETMOVER_HPP
# define TARGETMOVER_HPP

# include <string>
# include <vector>
# include <rclcpp/rclcpp.hpp>
# include <geometry_msgs/msg/point_stamped.hpp>
# include <ignition/transport/Node.hh>
# include <ignition/msgs/boolean.pb.h>
# include <ignition/msgs/pose.pb.h>

class TargetMover : public rclcpp::Node
{
	public:
		TargetMover(void);
		TargetMover(const TargetMover &other);
		TargetMover	&operator=(const TargetMover &other);
		~TargetMover(void);

	private:
		struct Waypoint
		{
			std::string	label;
			double		x;
			double		y;
			double		z;
			double		moveTime;
			double		holdTime;
		};

		struct State
		{
			double	x;
			double	y;
			double	z;
			size_t	stage;
			bool	moving;
		};

		std::string						_modelName;
		std::string						_worldName;
		double							_updateRate;
		bool							_loop;
		std::vector<Waypoint>			_path;
		double							_cycleTime;
		double							_startSec;
		bool							_started;
		double							_lastLogSec;
		rclcpp::TimerBase::SharedPtr	_timer;
		rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr	_gtPub;
		ignition::transport::Node		_gz;

		void	_loadParams(void);
		void	_loadPath(void);
		bool	_validatePath(void) const;
		void	_logPath(void) const;
		State	_makeState(size_t i, bool moving, double s) const;
		State	_stateAt(double t) const;
		void	_sendPose(const State &st);
		void	_onReply(const ignition::msgs::Boolean &rep, const bool result);
		void	_onTimer(void);
};

#endif
