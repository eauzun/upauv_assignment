#include <chrono>
#include <cmath>
#include <functional>
#include <stdexcept>
#include "TargetMover.hpp"

TargetMover::TargetMover(void)
	: rclcpp::Node("target_mover"), _updateRate(20.0), _loop(true),
	_cycleTime(0.0), _startSec(0.0), _started(false), _lastLogSec(0.0)
{
	this->_loadParams();
	if (!this->_validatePath())
		throw std::runtime_error("invalid target path configuration");
	this->_logPath();
	this->_gtPub = this->create_publisher<geometry_msgs::msg::PointStamped>(
		"/target_mover/ground_truth", 10);
	this->_timer = this->create_wall_timer(
		std::chrono::duration<double>(1.0 / this->_updateRate),
		std::bind(&TargetMover::_onTimer, this));
	RCLCPP_INFO(this->get_logger(), "target_mover started");
}

TargetMover::TargetMover(const TargetMover &other)
	: rclcpp::Node("target_mover"), _modelName(other._modelName),
	_worldName(other._worldName), _updateRate(other._updateRate),
	_loop(other._loop), _path(other._path), _cycleTime(other._cycleTime),
	_startSec(other._startSec), _started(other._started),
	_lastLogSec(other._lastLogSec)
{
}

TargetMover	&TargetMover::operator=(const TargetMover &other)
{
	if (this != &other)
	{
		this->_modelName = other._modelName;
		this->_worldName = other._worldName;
		this->_updateRate = other._updateRate;
		this->_loop = other._loop;
		this->_path = other._path;
		this->_cycleTime = other._cycleTime;
		this->_startSec = other._startSec;
		this->_started = other._started;
		this->_lastLogSec = other._lastLogSec;
	}
	return (*this);
}

TargetMover::~TargetMover(void)
{
}

void	TargetMover::_loadParams(void)
{
	this->_modelName = this->declare_parameter<std::string>("model_name", "red_target");
	this->_worldName = this->declare_parameter<std::string>("world_name", "up_test_world");
	this->_updateRate = this->declare_parameter<double>("update_rate", 20.0);
	this->_loop = this->declare_parameter<bool>("loop", true);
	this->_loadPath();
}

void	TargetMover::_loadPath(void)
{
	std::vector<std::string>	labels;
	std::vector<double>			x;
	std::vector<double>			y;
	std::vector<double>			z;
	std::vector<double>			moves;
	std::vector<double>			holds;
	Waypoint					wp;
	size_t						i;

	labels = this->declare_parameter<std::vector<std::string>>("labels", std::vector<std::string>());
	x = this->declare_parameter<std::vector<double>>("x", std::vector<double>());
	y = this->declare_parameter<std::vector<double>>("y", std::vector<double>());
	z = this->declare_parameter<std::vector<double>>("z", std::vector<double>());
	moves = this->declare_parameter<std::vector<double>>("move_time", std::vector<double>());
	holds = this->declare_parameter<std::vector<double>>("hold_time", std::vector<double>());
	if (labels.size() != x.size() || y.size() != x.size() || z.size() != x.size()
		|| moves.size() != x.size() || holds.size() != x.size())
	{
		RCLCPP_ERROR(this->get_logger(), "path arrays must all have the same length");
		return ;
	}
	i = 0;
	while (i < x.size())
	{
		wp.label = labels[i];
		wp.x = x[i];
		wp.y = y[i];
		wp.z = z[i];
		wp.moveTime = moves[i];
		wp.holdTime = holds[i];
		this->_path.push_back(wp);
		if (i > 0)
			this->_cycleTime += wp.moveTime;
		this->_cycleTime += wp.holdTime;
		i++;
	}
}

bool	TargetMover::_validatePath(void) const
{
	size_t	i;

	if (this->_modelName.empty() || this->_worldName.empty())
	{
		RCLCPP_ERROR(this->get_logger(), "model_name and world_name must not be empty");
		return (false);
	}
	if (this->_updateRate <= 0.0)
	{
		RCLCPP_ERROR(this->get_logger(), "update_rate must be > 0");
		return (false);
	}
	if (this->_path.size() < 2)
	{
		RCLCPP_ERROR(this->get_logger(), "path needs at least 2 waypoints");
		return (false);
	}
	i = 0;
	while (i < this->_path.size())
	{
		if (this->_path[i].holdTime < 0.0 || (i > 0 && this->_path[i].moveTime <= 0.0))
		{
			RCLCPP_ERROR(this->get_logger(),
				"waypoint %zu: hold_time must be >= 0 and move_time > 0", i);
			return (false);
		}
		i++;
	}
	if (this->_loop && (std::fabs(this->_path.front().x - this->_path.back().x) > 1e-6
		|| std::fabs(this->_path.front().y - this->_path.back().y) > 1e-6
		|| std::fabs(this->_path.front().z - this->_path.back().z) > 1e-6))
	{
		RCLCPP_ERROR(this->get_logger(), "loop is on but last waypoint differs from the first");
		return (false);
	}
	return (true);
}

void	TargetMover::_logPath(void) const
{
	size_t	i;

	RCLCPP_INFO(this->get_logger(), "model=%s world=%s rate=%.1f Hz loop=%s",
		this->_modelName.c_str(), this->_worldName.c_str(),
		this->_updateRate, this->_loop ? "true" : "false");
	i = 0;
	while (i < this->_path.size())
	{
		RCLCPP_INFO(this->get_logger(),
			"  [%zu] %-14s pos=(%.2f, %.2f, %.2f) move=%.1fs hold=%.1fs",
			i, this->_path[i].label.c_str(),
			this->_path[i].x, this->_path[i].y, this->_path[i].z,
			this->_path[i].moveTime, this->_path[i].holdTime);
		i++;
	}
	RCLCPP_INFO(this->get_logger(), "cycle time: %.1f s", this->_cycleTime);
}

TargetMover::State	TargetMover::_makeState(size_t i, bool moving, double s) const
{
	State	st;
	size_t	from;

	from = (moving && i > 0) ? i - 1 : i;
	st.stage = i;
	st.moving = moving;
	st.x = this->_path[from].x + (this->_path[i].x - this->_path[from].x) * s;
	st.y = this->_path[from].y + (this->_path[i].y - this->_path[from].y) * s;
	st.z = this->_path[from].z + (this->_path[i].z - this->_path[from].z) * s;
	return (st);
}

TargetMover::State	TargetMover::_stateAt(double t) const
{
	double	acc;
	size_t	i;

	if (this->_loop)
		t = std::fmod(t, this->_cycleTime);
	acc = this->_path[0].holdTime;
	if (t < acc)
		return (this->_makeState(0, false, 0.0));
	i = 1;
	while (i < this->_path.size())
	{
		if (t < acc + this->_path[i].moveTime)
			return (this->_makeState(i, true, (t - acc) / this->_path[i].moveTime));
		acc += this->_path[i].moveTime;
		if (t < acc + this->_path[i].holdTime)
			return (this->_makeState(i, false, 0.0));
		acc += this->_path[i].holdTime;
		i++;
	}
	return (this->_makeState(this->_path.size() - 1, false, 0.0));
}

void	TargetMover::_onTimer(void)
{
	double	now;
	double	t;
	State	st;

	now = this->get_clock()->now().seconds();
	if (!this->_started)
	{
		if (now <= 0.0)
			return ;
		this->_startSec = now;
		this->_lastLogSec = now - 1.0;
		this->_started = true;
	}
	t = now - this->_startSec;
	st = this->_stateAt(t);
	this->_sendPose(st);
	if (now - this->_lastLogSec >= 1.0)
	{
		this->_lastLogSec = now;
		RCLCPP_INFO(this->get_logger(),
			"t=%5.1f stage=%zu %-14s %s pos=(%.2f, %.2f, %.2f)",
			t, st.stage, this->_path[st.stage].label.c_str(),
			st.moving ? "moving " : "holding", st.x, st.y, st.z);
	}
}

void	TargetMover::_onReply(const ignition::msgs::Boolean &rep, const bool result)
{
	(void)rep;
	(void)result;
}

void	TargetMover::_sendPose(const State &st)
{
	ignition::msgs::Pose				req;
	geometry_msgs::msg::PointStamped	gt;

	req.set_name(this->_modelName);
	req.mutable_position()->set_x(st.x);
	req.mutable_position()->set_y(st.y);
	req.mutable_position()->set_z(st.z);
	req.mutable_orientation()->set_w(1.0);
	this->_gz.Request("/world/" + this->_worldName + "/set_pose",
		req, &TargetMover::_onReply, this);
	gt.header.stamp = this->get_clock()->now();
	gt.header.frame_id = "world";
	gt.point.x = st.x;
	gt.point.y = st.y;
	gt.point.z = st.z;
	this->_gtPub->publish(gt);
}
