#ifndef REDTARGETDETECTOR_HPP
# define REDTARGETDETECTOR_HPP

# include <chrono>
# include <string>
# include <vector>
# include <opencv2/core.hpp>
# include <rclcpp/rclcpp.hpp>
# include <geometry_msgs/msg/point_stamped.hpp>
# include <sensor_msgs/msg/image.hpp>
# include <std_msgs/msg/bool.hpp>
# include <vision_msgs/msg/detection2_d_array.hpp>

class RedTargetDetector : public rclcpp::Node
{
	public:
		RedTargetDetector(void);
		RedTargetDetector(const RedTargetDetector &other);
		RedTargetDetector	&operator=(const RedTargetDetector &other);
		~RedTargetDetector(void);

	private:
		struct Config
		{
			int		h1Lo;
			int		h1Hi;
			int		h2Lo;
			int		h2Hi;
			int		sMin;
			int		vMin;
			double	minArea;
			double	maxAreaRatio;
			int		kernel;
			int		confirmFrames;
			int		lostFrames;
			double	timeout;
			bool	debug;
		};

		struct Candidate
		{
			cv::Rect	box;
			double		area;
		};

		Config								_cfg;
		bool								_detected;
		int									_seen;
		int									_miss;
		std::vector<Candidate>				_lastCands;
		bool								_gotFrame;
		bool								_stale;
		std::chrono::steady_clock::time_point	_lastFrame;
		size_t								_statFrames;
		double								_statMs;
		double								_lastCx;
		double								_lastCy;
		double								_lastArea;

		rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr				_sub;
		rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr		_detPub;
		rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr						_boolPub;
		rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr			_offPub;
		rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr					_dbgPub;
		rclcpp::TimerBase::SharedPtr											_logTimer;
		rclcpp::TimerBase::SharedPtr											_dogTimer;
		rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr		_paramCb;

		void	_setDefaults(void);
		void	_declareParams(void);
		void	_loadParams(void);
		bool	_applyParam(const rclcpp::Parameter &p, Config &c, std::string &why) const;
		rcl_interfaces::msg::SetParametersResult
				_onParams(const std::vector<rclcpp::Parameter> &params);
		void	_findCandidates(const cv::Mat &bgr, std::vector<Candidate> &out) const;
		void	_updateState(bool raw);
		void	_publish(const std_msgs::msg::Header &hdr, cv::Mat &img,
					const std::vector<Candidate> &cands);
		void	_drawDebug(cv::Mat &img, const std::vector<Candidate> &out,
					const std::vector<Candidate> &cands) const;
		void	_onImage(const sensor_msgs::msg::Image::ConstSharedPtr msg);
		void	_onLog(void);
		void	_onWatchdog(void);
};

#endif
