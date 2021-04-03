#pragma once

#include <iostream>
#include <frc/Joystick.h>
#include "ctre/Phoenix.h"
#include "AHRS.h"
#include <thread>
#include <chrono>
#include <cmath>
#include <stdio.h>
#include <stdlib.h>
#include <cstdlib>
#include <frc/Timer.h>
#include <vector>
#include <list>
#include <frc/PWMTalonFx.h>


#include "DriveConstants.h"
#include "Constants.h"

using namespace frc;

class DriveBase {
public:


	WPI_TalonFX *canTalonLeft1, *canTalonLeft2, *canTalonRight1, *canTalonRight2;

	AHRS *ahrs;


	//HDrive
	// DriveBase(int fl, int fr, int rl, int rr, int k, int pcm, int f_channel, int r_channel);

	//WestCoast, 2-speed transmission option
	DriveBase(int l1, int l2, int r1, int r2, int pcm, int f_channel, int r_channel, bool two_speed);

	void ShiftUp();
	void ShiftDown();
	void SetGainsHigh();
	void SetGainsLow();

	void ResetConfigs();
	

	void ManualOpenLoopDrive(Joystick* throttle, Joystick* wheel);

	void AutoShift(bool auto_shift);

	void RunTeleopDrive(Joystick *JoyThrottle, Joystick *JoyWheel, bool is_regular, bool is_vision, bool is_rotation);
	void InitDriveProf();
	//void SelectAutonProfile();

	//Driving Operators
	void TeleopHDrive(Joystick *JoyThrottle, Joystick *JoyWheel, bool *is_fc); //creates velocity references set by joysticks, for HDrive Train
	void TeleopWCDrive(Joystick *JoyThrottle, Joystick *JoyWheel, bool pos_yaw, bool vis_yaw); //creates velocity references based on joysticks, for normal west coast drive train

	void RotationController(Joystick *JoyWheel);
	void Controller(double ref_kick, double ref_right, double ref_left,
		double ref_yaw, double k_p_right, double k_p_left, double k_p_kick,
		double k_p_yaw, double k_d_yaw, double k_d_right, double k_d_left,
		double k_d_kick, double target_vel_left, double target_vel_right,
		double target_vel_kick); //The final controller, will take the references set by either teleop or auton drive function

	//Motor Functions
	void ZeroAll(bool stop_motors);
	void StopAll();
	void ZeroEncs();
	void ZeroYaw();
	void ZeroI();

	double GetLeftPosition();
	double GetRightPosition();
	double GetYawPos();
	double GetLeftVel();
	double GetRightVel();
	double GetRoll();

	void SetMaxRpm(double rpm);
	double GetMaxRpm();

	//Function to fill the profile points vector for autonomous

	void SetZeroingIndex(std::vector<int> zero_index);

	//Wrapper Functions
	static void DriveWrapper(Joystick *JoyThrottle, Joystick *JoyWheel, bool *is_heading, bool *is_vision, bool *is_fc, DriveBase *driveController);

	//Auton functions for threads are in derived class

	//Thread Functions
	void StartDriveThreads(Joystick *JoyThrottle, Joystick *JoyWheel,
	bool *is_heading, bool *is_vision, bool *is_fc);
	void EndDriveThreads();

	//AutonThread functions for use in the daughter class
	void UpdateIndex();
	void ResetIndex();

	bool IsLastIndex();
	int GetDriveIndex();

	bool set_profile = false; //check to start auton drive

	bool is_vision_done = false;

private:

	double P_LEFT_VEL = 0;
	double D_LEFT_VEL = 0;
	double d_left_vel = 0;

	double P_RIGHT_VEL = 0;
	double D_RIGHT_VEL = 0;
	double d_right_vel = 0;

	double P_KICK_VEL = 0;
	double D_KICK_VEL = 0;
	double d_kick_vel = 0;

	double P_RIGHT_DIS = 0;
	double I_RIGHT_DIS = 0;
	double D_RIGHT_DIS = 0;

	double P_LEFT_DIS = 0;
	double I_LEFT_DIS = 0;
	double D_LEFT_DIS = 0;

	double P_KICK_DIS = 0;
	double I_KICK_DIS = 0;
	double D_KICK_DIS = 0;

	double P_YAW_DIS = 0;
	double I_YAW_DIS = 0;

	double d_vision = 0;

	double d_right = 0;
	double i_right = 0;

	double d_yaw_dis = 0;

	double d_left = 0;
	double i_left = 0;

	double d_kick = 0;
	double i_kick = 0;

	double i_yaw = 0;

	double l_error_vel_au = 0;
	double l_error_dis_au = 0;
	double r_error_vel_au = 0;
	double r_error_dis_au = 0;
	double k_error_dis_au = 0;
	double y_error_dis_au = 0; // yaw (theta) position error

	double l_error_vel_t = 0;
	double l_error_dis_t = 0;
	double r_error_vel_t = 0;
	double r_error_dis_t = 0;
	double kick_error_vel = 0;

	double l_last_error = 0;
	double r_last_error = 0;
	double yaw_last_error = 0;
	double kick_last_error = 0;

	double l_last_error_vel = 0;
	double r_last_error_vel = 0;
	double kick_last_error_vel = 0;

	int next_zero_index = 0;

	int zero_counter = 0;
	int zero_wait_counter = 0;

	double max_y_rpm, actual_max_y_rpm, max_yaw_rate;

	double k_p_right_vel, k_p_left_vel, k_p_yaw_vel, k_d_right_vel,
	k_d_left_vel, k_d_yaw_vel, k_p_kick_vel, k_d_kick_vel, k_p_yaw_h_vel,
	k_p_yaw_au, k_d_yaw_au;
	double k_p_yaw_heading_pos, k_d_vision_pos;
	double k_f_left_vel, k_f_right_vel;

	double actual_max_y_rpm_auton, actual_max_y_rpm_l_f, actual_max_y_rpm_l_b, actual_max_y_rpm_r_f, actual_max_y_rpm_r_b;

	double DYN_MAX_Y_RPM = 0;

	double k_p_yaw_dis, k_i_yaw_dis, k_d_yaw_dis, ff_scale;

	double k_p_right_vel_au = 0.0;
	double k_p_left_vel_au = 0.0;
	double k_p_kick_vel_au = 0.0;
	double k_d_left_vel_au = 0.0;
	double k_d_right_vel_au = 0.0;
	double k_d_kick_vel_au = 0.0;

	double D_YAW_DIS = 0.0;

	double l_last_current;

	double Kv;

	double max_vel_vis, max_acc_vis, max_jerk_vis;

	bool is_last_index = false;
	bool continue_profile = true;
	bool set_refs = false;
	int row_index = 0;
	int zeroing_counter = 0;
	std::vector<double> auton_row = { 0.0, 0.0, 0.0, 0.0, 0.0 }; //AutonDrive, row, individual points
	std::vector<int> zeroing_index;

	double feed_forward_r, feed_forward_l, feed_forward_k;

	bool is_zero;

	double init_heading = 0;
	double total_heading = 0;

	bool tank = false;
	bool is_low_gear = true;

	int LF = 0, L2 = 0, L3 = 0, LR = 0, RF = 0, R2 = 0, R3 = 0, RR = 0, KICKER = 0, PCM = 0, F_CHANNEL = 0, R_CHANNEL;

};

// const double TICKS_TO_DISTANCE = 1.0 / (2048.0 * (84.0 / 8.0) * (1 / (6 * apes::PI)) * (1 / 0.0254));
const double TICKS_TO_DISTANCE = 2.226463581265717488e-05;

double getSpeedFromTicksPer100Milliseconds(int ticks_per_second);
double getDistanceFromTicks(int ticks);
