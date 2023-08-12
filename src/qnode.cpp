#include "qnode.hpp"

namespace px4_gcs 
{

QNode::QNode(int argc, char** argv ) : init_argc(argc), init_argv(argv)
{
}

QNode::~QNode() 
{
	if(ros::isStarted()) 
	{
		ros::shutdown(); // explicitly needed since we use ros::start();
		ros::waitForShutdown();
    }
	wait();
}

bool QNode::init()
{
	if(!ros_flag_)
	{
		ros::init(init_argc,init_argv,"px4_gcs");
		if (!ros::master::check())
		{
			Q_EMIT emit_pushButton_connect_ros_color(false);
			return false;
		}
		ros_flag_ = true;

		ros::start(); // explicitly needed since our nodehandle is going out of scope.
		
		ros::NodeHandle n;
		ros::NodeHandle pn( ros::this_node::getName().c_str() );
	
		std::string ns = ros::this_node::getNamespace(); 
		if( ns.length()>1 )
			ns = ns.substr(1,ns.length());

		t_init_ = ros::Time::now();

		// parse name
		std::string name;
		pn.getParam( "name", name );
		Q_EMIT emit_window_title( name.c_str() );

		// parse topic address
		std::string nav;
		pn.getParam( "navigation", nav ); 
		nav = ns + "/" + nav; // name of navigation msg which can be different

		// Subscription	
		//sub_[0] = n.subscribe<nav_msgs::Odometry>
		//	(nav.c_str(), 10, &QNode::odom_cb, this);
		//sub_[1] = n.subscribe<mavros_msgs::AttitudeTarget>
		//	("mavros/setpoint_raw/attitude", 10, &QNode::att_sp_cb, this);
		//sub_[2] = n.subscribe<mavros_msgs::PositionTarget>
		//	("commander/setpoint_raw/position", 10, &QNode::pos_sp_cb, this);
		sub_[3] = n.subscribe<mavros_msgs::State>
			("mavros/state",10,&QNode::px4_state_cb,this);
		//sub_[4] = n.subscribe<sensor_msgs::Imu>
		//	("mavros/imu/data", 10, &QNode::imu_cb, this);
		//sub_[5] = n.subscribe<mavros_msgs::RCOut>
		//	("mavros/rc/out", 10, &QNode::rc_out_cb, this);
		//sub_[6] = n.subscribe<mavros_sgs::RCIn>
		//	("mavros/rc/in", 10, &QNode::rc_in_cb, this);
		sub_[8] = n.subscribe<keyboard::Key>
			("gcs/keyinput", 10, &QNode::key_cb, this);

		pub_[0] = n.advertise<keyboard::Key>("gcs/keyoutput", 10);

		// Services
		srv_client_[0] = n.serviceClient<std_srvs::SetBool>
			("commander/arm");
		srv_client_[1] = n.serviceClient<std_srvs::SetBool>
			("commander/flight_mode");
		srv_client_[2] = n.serviceClient<mavros_msgs::SetMode>
			("commander/control_mode");
		srv_client_[3] = n.serviceClient<mavros_msgs::CommandInt>
			("commander/move_setpoint");
		srv_client_[4] = n.serviceClient<std_srvs::SetBool>
			("logger/start");
		srv_client_[5] = n.serviceClient<std_srvs::SetBool>
			("logger/stop");
		srv_client_[6] = n.serviceClient<std_srvs::SetBool>
			("logger/send");
		srv_client_[7] = n.serviceClient<std_srvs::SetBool>
			("commander/move_dxl");
		srv_client_[8] = n.serviceClient<std_srvs::SetBool>
			("commander/move_position");
		srv_client_[9] = n.serviceClient<std_srvs::SetBool>
			("ilqr/start");
		srv_client_[10] = n.serviceClient<std_srvs::SetBool>
			("ilqr/visualization/start");
		srv_client_[11] = n.serviceClient<std_srvs::SetBool>
			("controller/use_trajectory");
		
		// ADDED
		srv_client_[12] = n.serviceClient<std_srvs::SetBool>
			("ref_planner/pos_sp_mode");
		// srv_client_[12] = n.serviceClient<std_srvs::SetBool>
		// 	("commander/DJ_useiLQR");
		// srv_client_[13] = n.serviceClient<std_srvs::SetBool>
		// 	("commander/DJ_stopiLQR");
		// ADDED - CHANGED: b/c dynamixel generates this message
		// srv_client_[14] = n.serviceClient<std_srvs::SetBool>
		// 	("dynamixel/DJ_initialConfig");

		// ADDED - JH
		srv_client_[13] = n.serviceClient<mavros_msgs::CommandInt>
			("am_controller/dynamixel_setpoint");

		// parse kill switch channel
		pn.getParam("kill_switch_ch", kill_switch_ch_);

		start();

		Q_EMIT emit_pushButton_connect_ros_color(true);
		return true;
	}
}

void QNode::run() 
{
	double rate = 30.0;
	ros::Rate loop_rate(rate);

	while( ros::ok() )
	{
		ros::spinOnce();
		if(init_flag_)
		{
			px4_timer_ += 1.0/rate;
			if( (px4_timer_ > PX4_LOSS_TIME) && !px4_signal_loss_ )
			{
				Q_EMIT emit_pushButton_connect_ros_color(false);
				px4_flag_ = false;
				px4_signal_loss_ = true;
			}
			if( (px4_timer_ < PX4_LOSS_TIME) && px4_signal_loss_ )
			{
				Q_EMIT emit_pushButton_connect_ros_color(true);
				px4_signal_loss_ = false;
			}
		}
		loop_rate.sleep();
	}
	Q_EMIT ros_shutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

/** subscription callbacks **/
void QNode::px4_state_cb(const mavros_msgs::State::ConstPtr &msg)
{
	if( !init_flag_ ) 
		init_flag_ = true;

	px4_state_ = *msg;
	px4_flag_ = msg->connected;
	
	if(px4_flag_)
		px4_timer_ = 0.0;
	
	if(init_flag_)
	{
		Q_EMIT emit_arming_state( px4_state_.armed );
		Q_EMIT emit_flight_mode( px4_state_.mode.c_str() );
	}
}

void QNode::att_sp_cb(const mavros_msgs::AttitudeTarget::ConstPtr &msg)
{
	if(init_flag_)
	{
		double roll, pitch, yaw;
		q2e( msg->orientation, roll, pitch, yaw);
		
		double* buf = (double*)malloc(6*sizeof(double));
		buf[1] = (180.0/3.14)*roll;
		buf[2] = (180.0/3.14)*pitch;
		buf[3] = (180.0/3.14)*msg->body_rate.x;	
		buf[4] = (180.0/3.14)*msg->body_rate.y; 
		buf[5] = (180.0/3.14)*msg->body_rate.z;
		buf[0] = now();
		
		Q_EMIT emit_attitude_setpoint( buf, (msg->header.seq % 10) == 0 );
	}
}

void QNode::pos_sp_cb(const mavros_msgs::PositionTarget::ConstPtr &msg)
{
	if(init_flag_)
	{
		double* buf = (double*)malloc(9*sizeof(double));	
		buf[1] = msg->position.x;
		buf[2] = msg->position.y;
		buf[3] = msg->position.z;
		buf[4] = msg->velocity.x;
		buf[5] = msg->velocity.y;
		buf[6] = msg->velocity.z;
		buf[7] = msg->yaw*(180.0/3.14);
		buf[8] = msg->yaw_rate*(180.0/3.14);
		buf[0] = now();
		
		Q_EMIT emit_position_setpoint( buf, msg->type_mask, 
				(msg->header.seq % 30) == 0 );
	}
}

void QNode::odom_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
	odom_ = *msg;

	if(init_flag_)
	{
		if( (msg->header.seq % 10) == 0 )
		{
			double roll, pitch, yaw;
			q2e( msg->pose.pose.orientation, roll, pitch, yaw);
			
			Eigen::Quaternion<double> q;
			q.w() = msg->pose.pose.orientation.w;
			q.x() = msg->pose.pose.orientation.x;
			q.y() = msg->pose.pose.orientation.y;
			q.z() = msg->pose.pose.orientation.z;
			Eigen::Matrix<double,3,3> R = q.toRotationMatrix();

			double* buf = (double*)malloc(13*sizeof(double));	
			
			buf[1] = msg->pose.pose.position.x;
			buf[2] = msg->pose.pose.position.y;
			buf[3] = msg->pose.pose.position.z;
		
			Eigen::Matrix<double,3,1> v_b;
			v_b(0,0) = msg->twist.twist.linear.x;
			v_b(1,0) = msg->twist.twist.linear.y;
			v_b(2,0) = msg->twist.twist.linear.z;
			Eigen::Matrix<double,3,1> v = R*v_b; // this is now pos_rate
			buf[4] = v(0,0); 
			buf[5] = v(1,0);
			buf[6] = v(2,0);

			buf[7] = roll*(180.0/3.14);
			buf[8] = pitch*(180.0/3.14);
			buf[9] = yaw*(180.0/3.14);

			buf[0] = now();

			Q_EMIT emit_navigation_state( buf );
		}
	}
}

void QNode::imu_cb(const sensor_msgs::Imu::ConstPtr &msg)
{
	if(init_flag_)
	{
		if( msg->header.seq % 10 == 0)
		{
			double* buf = (double*)malloc(7*sizeof(double));
			buf[0] = now();
	
			double phi, theta, psi;
			q2e( msg->orientation, phi, theta, psi);
	
			buf[1] = (180.0/3.14)*phi;
			buf[2] = (180.0/3.14)*theta;
			buf[3] = (180.0/3.14)*psi;

			Eigen::Matrix<double,3,1> w_b;
			w_b(0,0) = msg->angular_velocity.x;
			w_b(1,0) = msg->angular_velocity.y;
			w_b(2,0) = msg->angular_velocity.z;
			Eigen::Matrix<double,3,3> T;
			T(0,0) = 1.0; T(0,1) = sin(phi)*tan(theta); T(0,2) = cos(phi)*tan(theta);
			T(1,0) = 0.0; T(1,1) = cos(phi); 			T(1,2) = -sin(phi);
			T(2,0) = 0.0; T(2,1) = sin(phi)/cos(theta); T(2,2) = cos(phi)/cos(theta);
			Eigen::Matrix<double,3,1> euler_rate = T * w_b;

			buf[4] = (180.0/3.14)*euler_rate(0,0);
			buf[5] = (180.0/3.14)*euler_rate(1,0);
			buf[6] = (180.0/3.14)*euler_rate(2,0);
	
			Q_EMIT emit_imu_state( buf );
		}
	}
}

void QNode::key_cb(const keyboard::Key::ConstPtr &msg)
{
	Q_EMIT emit_keyinput( msg->code );
}

void QNode::rc_out_cb(const mavros_msgs::RCOut::ConstPtr &msg)
{
	if( init_flag_ )
	{
		double* buf = (double*)malloc(6*sizeof(double));
		for(int i=0; i<6; i++)
			buf[i] = msg->channels[i];

		Q_EMIT emit_pwms( buf );
	}
}

void QNode::rc_in_cb(const mavros_msgs::RCIn::ConstPtr &msg)
{
	if( init_flag_ )
	{
		int signal = msg->channels[kill_switch_ch_ - 1];
		if( signal > 1500 )
			Q_EMIT emit_kill_switch_enabled( true );
		else
			Q_EMIT emit_kill_switch_enabled( false );
	}
}

void q2e(const double q0, const double q1, const double q2, const double q3, 
		double& roll, double& pitch, double& yaw)
{
	roll = atan2(2*(q2*q3+q0*q1),q0*q0-q1*q1-q2*q2+q3*q3);
	pitch = atan2(-2*(q1*q3-q0*q2),sqrt((2*(q2*q3+q0*q1))*(2*(q2*q3+q0*q1))+(q0*q0-q1*q1-q2*q2+q3*q3)*(q0*q0-q1*q1-q2*q2+q3*q3)));
	yaw = atan2(2*(q1*q2+q0*q3),q0*q0+q1*q1-q2*q2-q3*q3);
}
void q2e(const geometry_msgs::Quaternion q, double& roll, double& pitch, double& yaw)
{
	double ysqr = q.y * q.y;

	double t0 = 2.0 * (q.w * q.x + q.y * q.z);
	double t1 = 1.0 - 2.0 * (q.x * q.x + ysqr);
	roll = std::atan2(t0, t1);

	double t2 = 2.0 * (q.w * q.y - q.z * q.x);
	t2 = ((t2 > 1.0) ? 1.0 : t2);
	t2 = ((t2 < -1.0) ? -1.0 : t2);
	pitch = std::asin(t2);

	double t3 = 2.0 * (q.w * q.z + q.x * q.y);
	double t4 = 1.0 - 2.0 * (ysqr + q.z * q.z);
	yaw = std::atan2(t3, t4);
}

}  // namespace px4_gcs
