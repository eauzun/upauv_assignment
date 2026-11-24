#ifndef TARGETMOVER_HPP
# define TARGETMOVER_HPP

# include <string>
# include <vector>
# include <rclcpp/rclcpp.hpp>

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

		std::string						_modelName;
		std::string						_worldName;
		double							_updateRate;
		bool							_loop;
		std::vector<Waypoint>			_path;
		double							_cycleTime;
		double							_startSec;
		bool							_started;
		rclcpp::TimerBase::SharedPtr	_timer;

		void	_loadParams(void);
		void	_loadPath(void);
		bool	_validatePath(void) const;
		void	_logPath(void) const;
		void	_onTimer(void);
};

#endif
