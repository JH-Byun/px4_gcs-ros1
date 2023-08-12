#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "main_window.hpp"

namespace px4_gcs {

using namespace Qt;

ICSL_GCS::ICSL_GCS(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent),
	qnode(argc,argv)
{
	ui.setupUi(this);

	setWindowIcon(QIcon(":/images/ICSL.png"));
	QObject::connect(&qnode, SIGNAL(ros_shutdown()), this, SLOT(close()));
	QObject::connect(&qnode, SIGNAL(emit_pushButton_connect_ros_color(bool)), 
						  this, SLOT(set_pushButton_connect_ros_color(bool))); 
	//QObject::connect(&qnode, SIGNAL(emit_pushButton_connect_px4_color(bool)), 
	//					  this, SLOT(set_pushButton_connect_px4_color(bool))); 
	QObject::connect(&qnode, SIGNAL(emit_arming_state(bool)), 
						  this, SLOT(set_arming_state(bool)));
	QObject::connect(&qnode, SIGNAL(emit_flight_mode(const char*)), 
						  this, SLOT(set_flight_mode(const char*)));
	QObject::connect(&qnode, SIGNAL(emit_navigation_state(double*)), 
						  this, SLOT(set_navigation_state(double*))); 
	QObject::connect(&qnode, SIGNAL(emit_imu_state(double*)), 
						  this, SLOT(set_imu_state(double*))); 
	QObject::connect(&qnode, SIGNAL(emit_position_setpoint(double*, int, bool)), 
						  this, SLOT(set_position_setpoint(double*, int, bool))); 
	QObject::connect(&qnode, SIGNAL(emit_attitude_setpoint(double*, bool)), 
						  this, SLOT(set_attitude_setpoint(double*, bool))); 
	QObject::connect(&qnode, SIGNAL(emit_kill_switch_enabled(bool)), 
						  this, SLOT(set_kill_switch_enabled(bool))); 
	QObject::connect(&qnode, SIGNAL(emit_keyinput(int)), 
						  this, SLOT(set_keyinput(int))); 
	QObject::connect(&qnode, SIGNAL(emit_window_title(const char*)), 
						  this, SLOT(set_window_title(const char*))); 
	QObject::connect(&qnode, SIGNAL(emit_pwms(double*)), 
						  this, SLOT(set_pwms(double*))); 
	QObject::connect(&qnode, SIGNAL(emit_control_outs(double*)), 
						  this, SLOT(set_control_outs(double*))); 

	key_P = new QShortcut(Qt::Key_P, ui.centralwidget);
	key_O = new QShortcut(Qt::Key_O, ui.centralwidget);
	key_A = new QShortcut(Qt::Key_A, ui.centralwidget);
	key_D = new QShortcut(Qt::Key_D, ui.centralwidget);
	key_W = new QShortcut(Qt::Key_W, ui.centralwidget);
	key_S = new QShortcut(Qt::Key_S, ui.centralwidget);
	key_Z = new QShortcut(Qt::Key_Z, ui.centralwidget);
	key_X = new QShortcut(Qt::Key_X, ui.centralwidget);
	key_Space = new QShortcut(Qt::Key_Space, ui.centralwidget);
	key_Enter = new QShortcut(Qt::Key_Enter, ui.centralwidget);
	//
	key_G = new QShortcut(Qt::Key_G, ui.centralwidget);
	key_H = new QShortcut(Qt::Key_H, ui.centralwidget);
	key_T = new QShortcut(Qt::Key_T, ui.centralwidget);

	// ADDED
	key_R = new QShortcut(Qt::Key_R, ui.centralwidget); // ref_planner pos_sp_mode_cb
	// key_V = new QShortcut(Qt::Key_V, ui.centralwidget); // use iLQR
	// key_N = new QShortcut(Qt::Key_N, ui.centralwidget); // stop iLQR
	// key_R = new QShortcut(Qt::Key_R, ui.centralwidget); // robotic arm initial config.

	// ADDED - JH
	key_1 = new QShortcut(Qt::Key_1, ui.centralwidget);
	key_2 = new QShortcut(Qt::Key_2, ui.centralwidget);
	key_3 = new QShortcut(Qt::Key_3, ui.centralwidget);
	key_4 = new QShortcut(Qt::Key_4, ui.centralwidget);

	QObject::connect( key_P, SIGNAL(activated()), this, SLOT(on_btn_P_pressed()) );
	QObject::connect( key_O, SIGNAL(activated()), this, SLOT(on_btn_O_pressed()) );
	QObject::connect( key_A, SIGNAL(activated()), this, SLOT(on_btn_A_pressed()) );
	QObject::connect( key_D, SIGNAL(activated()), this, SLOT(on_btn_D_pressed()) );
	QObject::connect( key_W, SIGNAL(activated()), this, SLOT(on_btn_W_pressed()) );
	QObject::connect( key_S, SIGNAL(activated()), this, SLOT(on_btn_S_pressed()) );
	QObject::connect( key_Z, SIGNAL(activated()), this, SLOT(on_btn_Z_pressed()) );
	QObject::connect( key_X, SIGNAL(activated()), this, SLOT(on_btn_X_pressed()) );
	QObject::connect( key_Space, SIGNAL(activated()), this, SLOT(on_btn_Space_pressed()) );
	QObject::connect( key_Enter, SIGNAL(activated()), this, SLOT(on_btn_Enter_pressed()) );
	//
	QObject::connect( key_G, SIGNAL(activated()), this, SLOT(on_btn_G_pressed()) );
	//QObject::connect( key_H, SIGNAL(activated()), this, SLOT(on_btn_H_pressed()) );
	QObject::connect( key_T, SIGNAL(activated()), this, SLOT(on_btn_T_pressed()) );

	// ADDED
	QObject::connect( key_R, SIGNAL(activated()), this, SLOT(on_btn_R_pressed()) );
	// QObject::connect( key_V, SIGNAL(activated()), this, SLOT(on_btn_V_pressed()) );
	// QObject::connect( key_N, SIGNAL(activated()), this, SLOT(on_btn_N_pressed()) );
	// QObject::connect( key_R, SIGNAL(activated()), this, SLOT(on_btn_R_pressed()) );

	// ADDED - JH
	QObject::connect( key_1, SIGNAL(activated()), this, SLOT(on_btn_1_pressed()) );
	QObject::connect( key_2, SIGNAL(activated()), this, SLOT(on_btn_2_pressed()) );
	QObject::connect( key_3, SIGNAL(activated()), this, SLOT(on_btn_3_pressed()) );
	QObject::connect( key_4, SIGNAL(activated()), this, SLOT(on_btn_4_pressed()) );

	setupGraph();
}

ICSL_GCS::~ICSL_GCS()
{
	delete key_P;
	delete key_O;
	delete key_A;
	delete key_D;
	delete key_W;
	delete key_S;
	delete key_Z;
	delete key_X;
	delete key_Space;
	delete key_Enter;
	//
	delete key_G;
	delete key_H;
	delete key_T;

	// ADDED
	delete key_R;
	// delete key_V;
	// delete key_N;
	// delete key_R;

	// ADDED - JH
	delete key_1;
	delete key_2;
	delete key_3;
	delete key_4;
}

/** slots **/
void ICSL_GCS::set_pushButton_connect_ros_color(bool flag)
{
	if(flag)	ui.pushButton_connect_ros->setStyleSheet("background-color: rgba(0,255,0,128);");
	else		ui.pushButton_connect_ros->setStyleSheet("background-color: rgba(255,0,0,128);");
}

//void ICSL_GCS::set_pushButton_connect_px4_color(bool flag)
//{
//	if(flag)	ui.pushButton_connect_px4->setStyleSheet("background-color: rgba(0,255,0,128);");
//	else		ui.pushButton_connect_px4->setStyleSheet("background-color: rgba(255,0,0,128);");
//}

void ICSL_GCS::set_arming_state(bool armed)
{
	if(armed)
	{
		ui.pushButton_arming->setStyleSheet("background-color: rgba(255,0,0,128);");
		ui.pushButton_arming->setText( QString("Armed") );
	}
	else
	{
		ui.pushButton_arming->setStyleSheet("background-color: rgba(0,255,0,128);");
		ui.pushButton_arming->setText( QString("Disarmed") );
	}
}
void ICSL_GCS::set_flight_mode(const char* mode)
{
	if( !strcmp(mode, "OFFBOARD") )
	{	// offboard
		ui.pushButton_flight_mode->setText( QString("Offboard") );
		ui.pushButton_flight_mode->setStyleSheet("background-color: rgba(255,0,0,128);");	
	}
	else
	{	// else
		ui.pushButton_flight_mode->setText( QString("Manual") );
		ui.pushButton_flight_mode->setStyleSheet("background-color: rgba(0,255,0,128);");
	}
}
void ICSL_GCS::set_navigation_state(double* buf)
{
	graph[0] ->draw(buf[0], buf[1],  0);
	graph[1] ->draw(buf[0], buf[2],  0);
	graph[2] ->draw(buf[0], buf[3],  0);
	graph[3] ->draw(buf[0], buf[4],  0);
	graph[4] ->draw(buf[0], buf[5],  0);
	graph[5] ->draw(buf[0], buf[6],  0);
	graph[6] ->draw(buf[0], buf[7],  0);
	graph[7] ->draw(buf[0], buf[8],  0);
	graph[8] ->draw(buf[0], buf[9],  0);
}
void ICSL_GCS::set_position_setpoint(double* buf, int mode, bool throttle)
{
	if( mode & HOLD )
	{
		graph[2]->draw(buf[0], buf[3], 1); // z
		graph[3]->draw(buf[0], buf[4], 1); // vx
		graph[4]->draw(buf[0], buf[5], 1); // vy
		if( throttle )
		{
			ui.pushButton_mode_hold->setStyleSheet("background-color: rgba(0,255,0,128);");
			ui.pushButton_mode_pos->setStyleSheet("background-color: rgba(255,255,255,128);");
			ui.pushButton_mode_vel->setStyleSheet("background-color: rgba(255,255,255,128);");
			ui.pushButton_mode_traj->setStyleSheet("background-color: rgba(255,255,255,128);");
		}
	}
	else if( mode & POSITION )
	{
		graph[0]->draw(buf[0], buf[1], 1); // x
		graph[1]->draw(buf[0], buf[2], 1); // y
		graph[2]->draw(buf[0], buf[3], 1); // z
		if( throttle )
		{
			ui.pushButton_mode_hold->setStyleSheet("background-color: rgba(255,255,255,128);");
			ui.pushButton_mode_pos->setStyleSheet("background-color: rgba(0,255,0,128);");
			ui.pushButton_mode_vel->setStyleSheet("background-color: rgba(255,255,255,128);");
			ui.pushButton_mode_traj->setStyleSheet("background-color: rgba(255,255,255,128);");
		}
	}
	else if( mode & VELOCITY )
	{
		graph[3]->draw(buf[0], buf[4], 1); // vx
		graph[4]->draw(buf[0], buf[5], 1); // vy
		graph[5]->draw(buf[0], buf[6], 1); // vz
		if( throttle )
		{
			ui.pushButton_mode_hold->setStyleSheet("background-color: rgba(255,255,255,128);");
			ui.pushButton_mode_pos->setStyleSheet("background-color: rgba(255,255,255,128);");
			ui.pushButton_mode_vel->setStyleSheet("background-color: rgba(0,255,0,128);");
			ui.pushButton_mode_traj->setStyleSheet("background-color: rgba(255,255,255,128);");
		}	
	}
	else if( mode & TRAJECTORY )
	{
		graph[0]->draw(buf[0], buf[1], 1); // x
		graph[1]->draw(buf[0], buf[2], 1); // y
		graph[2]->draw(buf[0], buf[3], 1); // z
		graph[3]->draw(buf[0], buf[4], 1); // vx
		graph[4]->draw(buf[0], buf[5], 1); // vy
		graph[5]->draw(buf[0], buf[6], 1); // vz
		if( throttle )
		{
			ui.pushButton_mode_hold->setStyleSheet("background-color: rgba(255,255,255,128);");
			ui.pushButton_mode_pos->setStyleSheet("background-color: rgba(255,255,255,128);");
			ui.pushButton_mode_vel->setStyleSheet("background-color: rgba(255,255,255,128);");
			ui.pushButton_mode_traj->setStyleSheet("background-color: rgba(0,255,0,128);");
		}
	}
	else
	{
		std::cout << "unrecognized mode 1" << std::endl;
	}

	if( mode & YAW )
	{
		graph[8]->draw(buf[0], buf[7], 1); // yaw
		if( throttle )
		{
			ui.pushButton_mode_yaw->setStyleSheet("background-color: rgba(0,255,0,128);");
			ui.pushButton_mode_yaw_rate->setStyleSheet("background-color: rgba(255,255,255,128);");
		}
	}
	else if( mode & YAWRATE )
	{
		std::cout << "this mode is currently not supported" << std::endl;
		if( throttle )
		{
			ui.pushButton_mode_yaw->setStyleSheet("background-color: rgba(255,255,255,128);");
			ui.pushButton_mode_yaw_rate->setStyleSheet("background-color: rgba(0,255,0,128);");
		}
	}
	else
	{
		std::cout << "unrecognized mode 2" << std::endl;
	}
}

void ICSL_GCS::set_attitude_setpoint(double* buf, bool throttle)
{
	if( throttle )
	{
		graph[6] ->draw(buf[0], buf[1], 1); // roll
		graph[7] ->draw(buf[0], buf[2], 1); // pitch
		graph[9] ->draw(buf[0], buf[3], 1); // wx
		graph[10]->draw(buf[0], buf[4], 1); // wy
		graph[11]->draw(buf[0], buf[5], 1); // wz
	}
}

void ICSL_GCS::set_imu_state(double* buf)
{
	graph[6] ->draw(buf[0], buf[1], 2); // roll
	graph[7] ->draw(buf[0], buf[2], 2); // pitch
//	graph[8] ->draw(buf[0], buf[3], 2); // yaw
	graph[9] ->draw(buf[0], buf[4], 2); // wx
	graph[10]->draw(buf[0], buf[5], 2); // wy
	graph[11]->draw(buf[0], buf[6], 2); // wz
}

void ICSL_GCS::set_kill_switch_enabled(bool tf)
{
	if( tf )
	{
		ui.pushButton_kill_switch->setText( QString("Enabled") );
		ui.pushButton_kill_switch->setStyleSheet("background-color: rgba(0,255,0,128);");
	}
	else
	{
		ui.pushButton_kill_switch->setText( QString("Disabled") );
		ui.pushButton_kill_switch->setStyleSheet("background-color: rgba(255,0,0,128);");	
	}
}

void ICSL_GCS::set_keyinput(int key)
{
	switch( key )
	{
		case KEY_P: on_btn_P_pressed(); break;
		case KEY_O: on_btn_O_pressed(); break;
		case KEY_A: on_btn_A_pressed(); break;
		case KEY_D: on_btn_D_pressed(); break;
		case KEY_W: on_btn_W_pressed(); break;
		case KEY_S: on_btn_S_pressed(); break;
		case KEY_Z: on_btn_Z_pressed(); break;
		case KEY_X: on_btn_X_pressed(); break;
		case KEY_G: on_btn_G_pressed(); break;
		//case KEY_H: on_btn_H_pressed(); break;
		case KEY_T: on_btn_T_pressed(); break;
		// ADDED
		case KEY_R: on_btn_R_pressed(); break;
		case KEY_SPACE: on_btn_Space_pressed(); break;
		case KEY_ENTER: on_btn_Enter_pressed(); break;

		// ADDED - JH
		case KEY_1: on_btn_1_pressed(); break;
		case KEY_2: on_btn_2_pressed(); break;
		case KEY_3: on_btn_3_pressed(); break;
		case KEY_4: on_btn_4_pressed(); break;
		
		default: break;
	}
}

void ICSL_GCS::set_window_title(const char* name)
{
	setWindowTitle(QApplication::translate("ICSL_GCS", name, 0, QApplication::UnicodeUTF8));
}

void ICSL_GCS::set_pwms(double* buf)
{
	ui.pushButton_pwm1->setText(QString( std::to_string( (int)(buf[0]) ).c_str() ));
	ui.pushButton_pwm2->setText(QString( std::to_string( (int)(buf[1]) ).c_str() ));
	ui.pushButton_pwm3->setText(QString( std::to_string( (int)(buf[2]) ).c_str() ));
	ui.pushButton_pwm4->setText(QString( std::to_string( (int)(buf[3]) ).c_str() ));
	ui.pushButton_pwm5->setText(QString( std::to_string( (int)(buf[4]) ).c_str() ));
	ui.pushButton_pwm6->setText(QString( std::to_string( (int)(buf[5]) ).c_str() ));
}

void ICSL_GCS::set_control_outs(double* buf)
{
	ui.pushButton_tau_x->setText(QString( std::to_string( (int)(buf[0]*100.0) ).c_str() ));
	ui.pushButton_tau_y->setText(QString( std::to_string( (int)(buf[1]*100.0) ).c_str() ));
	ui.pushButton_tau_z->setText(QString( std::to_string( (int)(buf[2]*100.0) ).c_str() ));
	ui.pushButton_thrust->setText(QString( std::to_string( (int)(buf[3]*100.0) ).c_str()));
}

/** etcs **/
void ICSL_GCS::setupGraph()
{
	graph[0] = new DrawingModule( ui.widget_p_x );
	graph[0]->setYLims(-0.5, 0.5);

	graph[1] = new DrawingModule( ui.widget_p_y );
	graph[1]->setYLims(-0.5, 0.5);

	graph[2] = new DrawingModule( ui.widget_p_z );
	graph[2]->setYLims(-0.5, 0.5);

	graph[3] = new DrawingModule( ui.widget_v_x );
	graph[3]->setYLims(-0.5, 0.5);
	
	graph[4] = new DrawingModule( ui.widget_v_y );
	graph[4]->setYLims(-0.5, 0.5);
	
	graph[5] = new DrawingModule( ui.widget_v_z );
	graph[5]->setYLims(-0.5, 0.5);
	
	graph[6] = new DrawingModule( ui.widget_roll );
	graph[6]->setYLims(-10.0, 10.0);
	
	graph[7] = new DrawingModule( ui.widget_pitch );
	graph[7]->setYLims(-10.0, 10.0);
	
	graph[8] = new DrawingModule( ui.widget_yaw );
	graph[8]->setYLims(-10.0, 10.0);
	
	graph[9] = new DrawingModule( ui.widget_p );
	graph[9]->setYLims(-30.0, 30.0);
	
	graph[10] = new DrawingModule( ui.widget_q );
	graph[10]->setYLims(-30.0, 30.0);
	
	graph[11] = new DrawingModule( ui.widget_r );
	graph[11]->setYLims(-30.0, 30.0);
}

}  // namespace px4_gcs
