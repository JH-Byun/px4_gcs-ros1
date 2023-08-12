#ifndef px4_gcs_QNODE_HPP_
#define px4_gcs_QNODE_HPP_

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <ros/network.h>
#endif

#include <keyboard/Key.h>

#include <string>
#include <QThread>
#include <Eigen/Dense>

#include <mavros_msgs/State.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/RCOut.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandInt.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_srvs/SetBool.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Joy.h>

#include "control_modes.h"

#include <thread>
#include <sstream>
#include <fstream>

#define PX4_LOSS_TIME 2.0

// ADDED - JH
#define PI 3.141592

namespace px4_gcs
{

	class QNode : public QThread
	{

		Q_OBJECT
	public:
		QNode(int argc, char **argv);
		virtual ~QNode();
		bool init();
		void run();

		// Command from main window
		void set_arm()
		{
			std_srvs::SetBool cmd;
			srv_client_[0].call(cmd);
		};
		void set_flight_mode()
		{
			std_srvs::SetBool cmd;
			srv_client_[1].call(cmd);
		};
		void set_ctrl_mode(ControlModes mode)
		{
			mavros_msgs::SetMode cmd;
			cmd.request.base_mode = mode;
			srv_client_[2].call(cmd);
		}
		void move_setpoint(int idx, bool increase)
		{
			mavros_msgs::CommandInt cmd;
			cmd.request.param1 = 0.0;
			cmd.request.param2 = 0.0;
			cmd.request.param3 = 0.0;
			cmd.request.param4 = 0.0;

			switch (idx)
			{
			case 0:
				increase ? cmd.request.param1 = 0.1 : cmd.request.param1 = -0.1;
				break;
			case 1:
				increase ? cmd.request.param2 = 0.1 : cmd.request.param2 = -0.1;
				break;
			case 2:
				increase ? cmd.request.param3 = 0.1 : cmd.request.param3 = -0.1;
				break;
			case 3:
				increase ? cmd.request.param4 = 3.141592 / 36 : cmd.request.param4 = -3.141592 / 36;
				break;
			default:
				break;
			}
			srv_client_[3].call(cmd);
		}
		void start_logger()
		{
			std_srvs::SetBool cmd;
			srv_client_[4].call(cmd);
		}
		void stop_logger()
		{
			std_srvs::SetBool cmd;
			srv_client_[5].call(cmd);
		}
		void send_logger()
		{
			std_srvs::SetBool cmd;
			srv_client_[6].call(cmd);
		}
		void move_arm()
		{
			std_srvs::SetBool cmd;
			srv_client_[7].call(cmd);
		}
		void move_position()
		{
			std_srvs::SetBool cmd;
			srv_client_[8].call(cmd);
		}
		void track_reference()
		{
			std_srvs::SetBool cmd;
			srv_client_[9].call(cmd);
			srv_client_[10].call(cmd);
		}
		void use_trajectory()
		{
			std_srvs::SetBool cmd;
			srv_client_[11].call(cmd);
		}
		void pub_key(int key)
		{
			keyboard::Key msg;
			msg.header.stamp = ros::Time::now();
			msg.code = key;
			pub_[0].publish(msg);
		}

		// ADDED
		void DJ_use_pos_sp()
		{
			std_srvs::SetBool cmd;
			srv_client_[12].call(cmd);
		}
		// void DJ_useiLQR()
		// {std_srvs::SetBool cmd; srv_client_[12].call(cmd);}
		// void DJ_stopiLQR()
		// {std_srvs::SetBool cmd; srv_client_[13].call(cmd);}
		// ADDED -CHANGED
		// void DJ_robotarmInitialConfig()
		// {std_srvs::SetBool cmd; srv_client_[14].call(cmd);}

		// ADDED - JH
		void dynamixel_setpoint(int idx, bool increase)
		{
			mavros_msgs::CommandInt cmd;
			cmd.request.param1 = 0.0;
			cmd.request.param2 = 0.0;

			switch (idx)
			{
			case 1:
				increase ? cmd.request.param1 = 5 : cmd.request.param1 = -5;
				break;
			case 2:
				increase ? cmd.request.param2 = 5 : cmd.request.param2 = -5;
				break;
			default:
				break;
			}
			srv_client_[13].call(cmd);
		}

	Q_SIGNALS:
		// quit
		void ros_shutdown();

		// Signal to main window
		void emit_pushButton_connect_ros_color(bool);
		void emit_pushButton_connect_px4_color(bool);
		void emit_arming_state(bool);
		void emit_flight_mode(const char *);
		void emit_navigation_state(double *);
		void emit_imu_state(double *);
		void emit_position_setpoint(double *, int, bool);
		void emit_attitude_setpoint(double *, bool);
		void emit_kill_switch_enabled(bool);
		void emit_keyinput(int);
		void emit_window_title(const char *);
		void emit_pwms(double *);
		void emit_control_outs(double *);

	private:
		int init_argc;
		char **init_argv;
		double now() { return (ros::Time::now() - t_init_).toSec(); }

		ros::Time t_init_;

		/** subscriber and callbacks **/
		ros::Subscriber sub_[10];
		void px4_state_cb(const mavros_msgs::State::ConstPtr &);
		void att_sp_cb(const mavros_msgs::AttitudeTarget::ConstPtr &);
		void odom_cb(const nav_msgs::Odometry::ConstPtr &);
		void imu_cb(const sensor_msgs::Imu::ConstPtr &);
		void key_cb(const keyboard::Key::ConstPtr &);
		void rc_out_cb(const mavros_msgs::RCOut::ConstPtr &);
		void rc_in_cb(const mavros_msgs::RCIn::ConstPtr &);
		void pos_sp_cb(const mavros_msgs::PositionTarget::ConstPtr &);

		ros::Publisher pub_[10];

		/** current states **/
		mavros_msgs::State px4_state_;
		nav_msgs::Odometry odom_;

		/** service client **/
		ros::ServiceClient srv_client_[20];

		/** flags **/
		bool init_flag_ = false;
		bool ros_flag_ = false;
		bool px4_flag_ = false; // true if temporal treatment for DJI platforms
		bool px4_signal_loss_ = false;
		double px4_timer_ = 0.0;

		int kill_switch_ch_ = 0;
	};

	// utility functions
	void q2e(const double, const double, const double, const double, double &, double &, double &);
	void q2e(const geometry_msgs::Quaternion, double &, double &, double &);

} // namespace px4_gcs

#endif /* px4_gcs_QNODE_HPP_ */
