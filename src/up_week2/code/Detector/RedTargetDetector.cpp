#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include "RedTargetDetector.hpp"

static const char	*g_names[] = {
	"h1_low", "h1_high", "h2_low", "h2_high", "s_min", "v_min",
	"min_area", "max_area_ratio", "morph_kernel", "confirm_frames",
	"lost_frames", "stream_timeout", "publish_debug", NULL
};

RedTargetDetector::RedTargetDetector(void)
	: rclcpp::Node("red_target_detector"), _detected(false), _seen(0), _miss(0),
	_gotFrame(false), _stale(false), _statFrames(0), _statMs(0.0),
	_lastCx(0.0), _lastCy(0.0), _lastArea(0.0)
{
	this->_setDefaults();
	this->_paramCb = this->add_on_set_parameters_callback(
		std::bind(&RedTargetDetector::_onParams, this, std::placeholders::_1));
	this->_declareParams();
	this->_loadParams();
	this->_detPub = this->create_publisher<vision_msgs::msg::Detection2DArray>(
		"/vision/detections", 10);
	this->_boolPub = this->create_publisher<std_msgs::msg::Bool>(
		"/vision/target_detected", 10);
	this->_offPub = this->create_publisher<geometry_msgs::msg::PointStamped>(
		"/vision/target_offset", 10);
	this->_dbgPub = this->create_publisher<sensor_msgs::msg::Image>(
		"/vision/debug_image", rclcpp::QoS(1));
	this->_sub = this->create_subscription<sensor_msgs::msg::Image>(
		"/camera/image_raw", rclcpp::SensorDataQoS().keep_last(1),
		std::bind(&RedTargetDetector::_onImage, this, std::placeholders::_1));
	this->_logTimer = this->create_wall_timer(std::chrono::seconds(1),
		std::bind(&RedTargetDetector::_onLog, this));
	this->_dogTimer = this->create_wall_timer(std::chrono::milliseconds(100),
		std::bind(&RedTargetDetector::_onWatchdog, this));
	RCLCPP_INFO(this->get_logger(), "red_target_detector started");
}

RedTargetDetector::RedTargetDetector(const RedTargetDetector &other)
	: rclcpp::Node("red_target_detector"), _cfg(other._cfg),
	_detected(other._detected), _seen(other._seen), _miss(other._miss),
	_lastCands(other._lastCands), _gotFrame(other._gotFrame),
	_stale(other._stale), _lastFrame(other._lastFrame),
	_statFrames(other._statFrames), _statMs(other._statMs),
	_lastCx(other._lastCx), _lastCy(other._lastCy), _lastArea(other._lastArea)
{
}

RedTargetDetector	&RedTargetDetector::operator=(const RedTargetDetector &other)
{
	if (this != &other)
	{
		this->_cfg = other._cfg;
		this->_detected = other._detected;
		this->_seen = other._seen;
		this->_miss = other._miss;
		this->_lastCands = other._lastCands;
		this->_gotFrame = other._gotFrame;
		this->_stale = other._stale;
		this->_lastFrame = other._lastFrame;
		this->_statFrames = other._statFrames;
		this->_statMs = other._statMs;
		this->_lastCx = other._lastCx;
		this->_lastCy = other._lastCy;
		this->_lastArea = other._lastArea;
	}
	return (*this);
}

RedTargetDetector::~RedTargetDetector(void)
{
}

void	RedTargetDetector::_setDefaults(void)
{
	this->_cfg.h1Lo = 0;
	this->_cfg.h1Hi = 6;
	this->_cfg.h2Lo = 174;
	this->_cfg.h2Hi = 180;
	this->_cfg.sMin = 120;
	this->_cfg.vMin = 70;
	this->_cfg.minArea = 80.0;
	this->_cfg.maxAreaRatio = 0.5;
	this->_cfg.kernel = 3;
	this->_cfg.confirmFrames = 3;
	this->_cfg.lostFrames = 5;
	this->_cfg.timeout = 1.0;
	this->_cfg.debug = true;
}

void	RedTargetDetector::_declareParams(void)
{
	this->declare_parameter<int>("h1_low", this->_cfg.h1Lo);
	this->declare_parameter<int>("h1_high", this->_cfg.h1Hi);
	this->declare_parameter<int>("h2_low", this->_cfg.h2Lo);
	this->declare_parameter<int>("h2_high", this->_cfg.h2Hi);
	this->declare_parameter<int>("s_min", this->_cfg.sMin);
	this->declare_parameter<int>("v_min", this->_cfg.vMin);
	this->declare_parameter<double>("min_area", this->_cfg.minArea);
	this->declare_parameter<double>("max_area_ratio", this->_cfg.maxAreaRatio);
	this->declare_parameter<int>("morph_kernel", this->_cfg.kernel);
	this->declare_parameter<int>("confirm_frames", this->_cfg.confirmFrames);
	this->declare_parameter<int>("lost_frames", this->_cfg.lostFrames);
	this->declare_parameter<double>("stream_timeout", this->_cfg.timeout);
	this->declare_parameter<bool>("publish_debug", this->_cfg.debug);
}

void	RedTargetDetector::_loadParams(void)
{
	Config		c;
	std::string	why;
	size_t		i;

	c = this->_cfg;
	i = 0;
	while (g_names[i])
	{
		if (!this->_applyParam(this->get_parameter(g_names[i]), c, why))
			throw std::runtime_error("invalid parameter: " + why);
		i++;
	}
	this->_cfg = c;
}

bool	RedTargetDetector::_applyParam(const rclcpp::Parameter &p, Config &c,
	std::string &why) const
{
	std::string	n;
	int			iv;
	double		dv;

	n = p.get_name();
	if (n == "h1_low" || n == "h1_high" || n == "h2_low" || n == "h2_high")
	{
		iv = (int)p.as_int();
		if (iv < 0 || iv > 180)
			return (why = n + " must be in [0, 180]", false);
		if (n == "h1_low")
			c.h1Lo = iv;
		else if (n == "h1_high")
			c.h1Hi = iv;
		else if (n == "h2_low")
			c.h2Lo = iv;
		else
			c.h2Hi = iv;
	}
	else if (n == "s_min" || n == "v_min")
	{
		iv = (int)p.as_int();
		if (iv < 0 || iv > 255)
			return (why = n + " must be in [0, 255]", false);
		if (n == "s_min")
			c.sMin = iv;
		else
			c.vMin = iv;
	}
	else if (n == "min_area")
	{
		dv = p.as_double();
		if (dv < 0.0)
			return (why = "min_area must be >= 0", false);
		c.minArea = dv;
	}
	else if (n == "max_area_ratio")
	{
		dv = p.as_double();
		if (dv <= 0.0 || dv > 1.0)
			return (why = "max_area_ratio must be in (0, 1]", false);
		c.maxAreaRatio = dv;
	}
	else if (n == "morph_kernel")
	{
		iv = (int)p.as_int();
		if (iv < 1 || iv > 31 || iv % 2 == 0)
			return (why = "morph_kernel must be odd and in [1, 31]", false);
		c.kernel = iv;
	}
	else if (n == "confirm_frames" || n == "lost_frames")
	{
		iv = (int)p.as_int();
		if (iv < 1)
			return (why = n + " must be >= 1", false);
		if (n == "confirm_frames")
			c.confirmFrames = iv;
		else
			c.lostFrames = iv;
	}
	else if (n == "stream_timeout")
	{
		dv = p.as_double();
		if (dv <= 0.0)
			return (why = "stream_timeout must be > 0", false);
		c.timeout = dv;
	}
	else if (n == "publish_debug")
		c.debug = p.as_bool();
	return (true);
}

rcl_interfaces::msg::SetParametersResult
	RedTargetDetector::_onParams(const std::vector<rclcpp::Parameter> &params)
{
	rcl_interfaces::msg::SetParametersResult	res;
	Config										c;
	std::string									why;
	size_t										i;

	res.successful = true;
	c = this->_cfg;
	i = 0;
	while (i < params.size())
	{
		try
		{
			if (!this->_applyParam(params[i], c, why))
			{
				res.successful = false;
				res.reason = why;
				return (res);
			}
		}
		catch (const std::exception &e)
		{
			res.successful = false;
			res.reason = e.what();
			return (res);
		}
		i++;
	}
	this->_cfg = c;
	return (res);
}

void	RedTargetDetector::_findCandidates(const cv::Mat &bgr,
	std::vector<Candidate> &out) const
{
	cv::Mat									hsv;
	cv::Mat									m1;
	cv::Mat									m2;
	cv::Mat									mask;
	cv::Mat									ker;
	std::vector<std::vector<cv::Point> >	cnt;
	Candidate								c;
	double									img;
	size_t									i;

	cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);
	cv::inRange(hsv, cv::Scalar(this->_cfg.h1Lo, this->_cfg.sMin, this->_cfg.vMin),
		cv::Scalar(this->_cfg.h1Hi, 255, 255), m1);
	cv::inRange(hsv, cv::Scalar(this->_cfg.h2Lo, this->_cfg.sMin, this->_cfg.vMin),
		cv::Scalar(this->_cfg.h2Hi, 255, 255), m2);
	cv::bitwise_or(m1, m2, mask);
	ker = cv::getStructuringElement(cv::MORPH_ELLIPSE,
		cv::Size(this->_cfg.kernel, this->_cfg.kernel));
	cv::morphologyEx(mask, mask, cv::MORPH_OPEN, ker);
	cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, ker);
	cv::findContours(mask, cnt, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
	img = (double)bgr.rows * (double)bgr.cols;
	i = 0;
	while (i < cnt.size())
	{
		c.area = cv::contourArea(cnt[i]);
		if (c.area >= this->_cfg.minArea && c.area <= this->_cfg.maxAreaRatio * img)
		{
			c.box = cv::boundingRect(cnt[i]);
			out.push_back(c);
		}
		i++;
	}
	std::sort(out.begin(), out.end(),
		[](const Candidate &a, const Candidate &b) { return (a.area > b.area); });
}

void	RedTargetDetector::_updateState(bool raw)
{
	if (raw)
	{
		this->_seen++;
		this->_miss = 0;
	}
	else
	{
		this->_miss++;
		this->_seen = 0;
	}
	if (this->_seen > 1000000)
		this->_seen = 1000000;
	if (this->_miss > 1000000)
		this->_miss = 1000000;
	if (!this->_detected && this->_seen >= this->_cfg.confirmFrames)
		this->_detected = true;
	else if (this->_detected && this->_miss >= this->_cfg.lostFrames)
		this->_detected = false;
}

void	RedTargetDetector::_drawDebug(cv::Mat &img, const std::vector<Candidate> &out,
	const std::vector<Candidate> &cands) const
{
	cv::Point	ctr;
	size_t		i;

	if (!this->_detected)
	{
		i = 0;
		while (i < cands.size())
			cv::rectangle(img, cands[i++].box, cv::Scalar(0, 255, 255), 1);
	}
	i = 0;
	while (i < out.size())
	{
		ctr = cv::Point(out[i].box.x + out[i].box.width / 2,
			out[i].box.y + out[i].box.height / 2);
		cv::rectangle(img, out[i].box, cv::Scalar(0, 255, 0), 2);
		cv::drawMarker(img, ctr, cv::Scalar(255, 0, 0), cv::MARKER_CROSS, 16, 2);
		i++;
	}
	cv::putText(img, this->_detected ? "TARGET" : "no target", cv::Point(8, 20),
		cv::FONT_HERSHEY_SIMPLEX, 0.6,
		this->_detected ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255), 2);
}

void	RedTargetDetector::_publish(const std_msgs::msg::Header &hdr, cv::Mat &img,
	const std::vector<Candidate> &cands)
{
	vision_msgs::msg::Detection2DArray	arr;
	vision_msgs::msg::Detection2D		det;
	std_msgs::msg::Bool					flag;
	geometry_msgs::msg::PointStamped	off;
	std::vector<Candidate>				out;
	sensor_msgs::msg::Image::SharedPtr	dbg;
	double								w;
	double								h;
	size_t								i;

	if (this->_detected && !cands.empty())
	{
		out = cands;
		this->_lastCands = cands;
	}
	else if (this->_detected)
		out = this->_lastCands;
	w = (double)img.cols;
	h = (double)img.rows;
	arr.header = hdr;
	i = 0;
	while (i < out.size())
	{
		det.header = hdr;
		det.id = "red_target";
		det.bbox.center.position.x = out[i].box.x + out[i].box.width / 2.0;
		det.bbox.center.position.y = out[i].box.y + out[i].box.height / 2.0;
		det.bbox.size_x = out[i].box.width;
		det.bbox.size_y = out[i].box.height;
		arr.detections.push_back(det);
		i++;
	}
	this->_detPub->publish(arr);
	flag.data = this->_detected;
	this->_boolPub->publish(flag);
	if (!out.empty())
	{
		this->_lastCx = arr.detections[0].bbox.center.position.x;
		this->_lastCy = arr.detections[0].bbox.center.position.y;
		this->_lastArea = out[0].area;
		off.header = hdr;
		off.point.x = std::max(-1.0, std::min(1.0, (this->_lastCx - w / 2.0) / (w / 2.0)));
		off.point.y = std::max(-1.0, std::min(1.0, (this->_lastCy - h / 2.0) / (h / 2.0)));
		off.point.z = this->_lastArea / (w * h);
		this->_offPub->publish(off);
	}
	else
		this->_lastArea = 0.0;
	if (this->_cfg.debug)
	{
		this->_drawDebug(img, out, cands);
		dbg = cv_bridge::CvImage(hdr, "bgr8", img).toImageMsg();
		this->_dbgPub->publish(*dbg);
	}
}

void	RedTargetDetector::_onImage(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
	std::chrono::steady_clock::time_point	t0;
	std::chrono::steady_clock::time_point	t1;
	cv_bridge::CvImagePtr					cv;
	std::vector<Candidate>					cands;

	t0 = std::chrono::steady_clock::now();
	this->_lastFrame = t0;
	this->_gotFrame = true;
	if (this->_stale)
	{
		this->_stale = false;
		RCLCPP_INFO(this->get_logger(), "image stream resumed");
	}
	if (msg->width == 0 || msg->height == 0 || msg->data.empty()
		|| msg->data.size() < (size_t)msg->step * msg->height)
	{
		RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
			"empty or corrupt image ignored");
		return ;
	}
	try
	{
		cv = cv_bridge::toCvCopy(msg, "bgr8");
		if (cv->image.empty())
			return ;
		this->_findCandidates(cv->image, cands);
		this->_updateState(!cands.empty());
		this->_publish(msg->header, cv->image, cands);
	}
	catch (const std::exception &e)
	{
		RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
			"frame skipped: %s", e.what());
		return ;
	}
	t1 = std::chrono::steady_clock::now();
	this->_statFrames++;
	this->_statMs += std::chrono::duration<double, std::milli>(t1 - t0).count();
}

void	RedTargetDetector::_onLog(void)
{
	double	avg;

	avg = this->_statFrames ? this->_statMs / (double)this->_statFrames : 0.0;
	RCLCPP_INFO(this->get_logger(),
		"frames=%zu detected=%s center=(%.1f, %.1f) area=%.0f avg=%.2f ms",
		this->_statFrames, this->_detected ? "yes" : "no",
		this->_detected ? this->_lastCx : 0.0,
		this->_detected ? this->_lastCy : 0.0,
		this->_detected ? this->_lastArea : 0.0, avg);
	this->_statFrames = 0;
	this->_statMs = 0.0;
}

void	RedTargetDetector::_onWatchdog(void)
{
	std_msgs::msg::Bool	flag;
	double				idle;

	if (!this->_gotFrame)
		return ;
	idle = std::chrono::duration<double>(
		std::chrono::steady_clock::now() - this->_lastFrame).count();
	if (idle <= this->_cfg.timeout)
		return ;
	if (!this->_stale)
	{
		this->_stale = true;
		RCLCPP_WARN(this->get_logger(), "no image for %.1f s, target lost", idle);
	}
	this->_detected = false;
	this->_seen = 0;
	this->_miss = 0;
	this->_lastCands.clear();
	flag.data = false;
	this->_boolPub->publish(flag);
}
