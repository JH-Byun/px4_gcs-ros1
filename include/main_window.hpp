#ifndef px4_gcs_MAIN_WINDOW_H
#define px4_gcs_MAIN_WINDOW_H

#define QCUSTOMPLOT_USE_LIBRARY

#include <QtGui/QMainWindow>
#include <QtGui/QWidget>
#include <QObject>
#include <QShortcut>
#include "ui_v2_gcs_simple.h"
#include "qnode.hpp"

#include "modules/drawing_module.h"
#include "control_modes.h"
#include "keyboard_handler.h"

// ADDED
#include <iostream>

namespace px4_gcs 
{

class ICSL_GCS : public QMainWindow
{
	Q_OBJECT
	
	public:
		ICSL_GCS(int argc, char** argv, QWidget *parent = 0);
		~ICSL_GCS();
	
	public Q_SLOTS:
		// signal from QNode
		void set_pushButton_connect_ros_color(bool);
		//void set_pushButton_connect_px4_color(bool);
		void set_arming_state(bool);
		void set_flight_mode(const char*);
		void set_navigation_state(double*);
		void set_imu_state(double*);
		void set_position_setpoint(double*, int, bool);
		void set_attitude_setpoint(double*, bool);
		void set_kill_switch_enabled(bool);
		void set_keyinput(int);
		void set_window_title(const char*);
		void set_pwms(double*);
		void set_control_outs(double*);

		// keyboard interaction
		void on_btn_P_pressed(){ qnode.move_setpoint(2,true);  qnode.pub_key(KEY_P); }
		void on_btn_O_pressed(){ qnode.move_setpoint(2,false); qnode.pub_key(KEY_O); }
		void on_btn_A_pressed(){ qnode.move_setpoint(1,true);  qnode.pub_key(KEY_A);}
		void on_btn_D_pressed(){ qnode.move_setpoint(1,false); qnode.pub_key(KEY_D);}
		void on_btn_W_pressed(){ qnode.move_setpoint(0,true);  qnode.pub_key(KEY_W);}
		void on_btn_S_pressed(){ qnode.move_setpoint(0,false); qnode.pub_key(KEY_S);}
		void on_btn_Z_pressed(){ qnode.move_setpoint(3,true);  qnode.pub_key(KEY_Z);}
		void on_btn_X_pressed(){ qnode.move_setpoint(3,false); qnode.pub_key(KEY_X);}

		void on_btn_Space_pressed()
		{ on_pushButton_arming_clicked(); qnode.pub_key(KEY_SPACE); }
		void on_btn_Enter_pressed()
		{ on_pushButton_flight_mode_clicked(); qnode.pub_key(KEY_ENTER); }
		
		void on_btn_G_pressed(){ qnode.use_trajectory(); qnode.pub_key(KEY_G); }
		//void on_btn_H_pressed(){ qnode.move_position(); }
		void on_btn_T_pressed(){ qnode.track_reference(); qnode.pub_key(KEY_T); }

		// ADDED
		void on_btn_R_pressed() // to ref_planner
		{ qnode.DJ_use_pos_sp(); std::cout << "btn R clicked" << std::endl; }
		// void on_btn_V_pressed()
		// { qnode.DJ_useiLQR(); std::cout << "btn V clicked" << std::endl; }
		// void on_btn_N_pressed()
		// { qnode.DJ_stopiLQR(); std::cout << "btn N clicked" << std::endl; }
		// ADDED - CHANGED
		// void on_btn_R_pressed()
		// { qnode.DJ_robotarmInitialConfig(); std::cout << "btn R clicked" << std::endl; }
		
		// ADDED - JH
		void on_btn_1_pressed(){ qnode.dynamixel_setpoint(1, false); qnode.pub_key(KEY_1); }
		void on_btn_2_pressed(){ qnode.dynamixel_setpoint(1, true);  qnode.pub_key(KEY_2);}
		void on_btn_3_pressed(){ qnode.dynamixel_setpoint(2, false); qnode.pub_key(KEY_3);}
		void on_btn_4_pressed(){ qnode.dynamixel_setpoint(2, true);  qnode.pub_key(KEY_4);}

		// pushbutton interaction
		void on_pushButton_connect_ros_clicked(){ qnode.init(); }
		void on_pushButton_arming_clicked(){ qnode.set_arm(); }
		void on_pushButton_flight_mode_clicked(){ qnode.set_flight_mode(); }
		void on_pushButton_mode_hold_clicked(){ qnode.set_ctrl_mode( HOLD ); }
		void on_pushButton_mode_pos_clicked(){ qnode.set_ctrl_mode( POSITION ); };
		void on_pushButton_mode_vel_clicked(){ qnode.set_ctrl_mode( VELOCITY ); };
		void on_pushButton_mode_traj_clicked(){ qnode.set_ctrl_mode( TRAJECTORY ); };
		void on_pushButton_mode_yaw_clicked(){ qnode.set_ctrl_mode( YAW ); };
		void on_pushButton_mode_yaw_rate_clicked(){ qnode.set_ctrl_mode( YAWRATE ); };
		void on_pushButton_log_start_clicked(){ qnode.start_logger(); }
		void on_pushButton_log_stop_clicked(){ qnode.stop_logger(); }
		void on_pushButton_log_send_clicked(){ qnode.send_logger(); }

	private:
		Ui::ICSL_GCS ui;
		QNode qnode;
	
		DrawingModule* graph[25];
		
		QShortcut* key_P;
		QShortcut* key_O;
		QShortcut* key_A;
		QShortcut* key_D;
		QShortcut* key_W;
		QShortcut* key_S;
		QShortcut* key_Z;
		QShortcut* key_X;
		QShortcut* key_Space;
		QShortcut* key_Enter;
		// ADDED
		QShortcut* key_R;
		// QShortcut* key_V;
		// QShortcut* key_N;
		// QShortcut* key_R;

		// for test
		QShortcut* key_G;
		QShortcut* key_H;
		QShortcut* key_T;

		// ADDED - JH
		QShortcut* key_1;
		QShortcut* key_2;
		QShortcut* key_3;
		QShortcut* key_4;
		
		void setupGraph();
};

}  // namespace px4_gcs

#endif // px4_gcs_MAIN_WINDOW_H
