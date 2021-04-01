#include "Drive/DriveBase.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include <frc/RobotState.h>
#include "Macros.h"
#include "Constants.h"
#include "frc/RobotState.h"

using namespace std::chrono;

//teleop drive sm
const int REGULAR = 0;
const int VISION_DRIVE = 1;
const int ROTATION_CONTROLLER = 2;
int teleop_drive_state = REGULAR;
int last_drive_state = REGULAR;

//vision drive sm
const int CREATE_VIS_PROF = 0;
const int FOLLOW_VIS_PROF = 1;
const int RESET_VIS = 2;
int vision_drive_state = CREATE_VIS_PROF;

//auton drive sm
const int CREATE_AUTON_PROF = 0;
const int FOLLOW_AUTON_PROF = 1;
const int RESET_AUTON = 2;
const int TELEOP_DRIVE = 3;
int auton_drive_state = CREATE_AUTON_PROF;

std::vector<std::vector<double>> vision_profile; //runautondrive looks at auton_row size
std::vector<std::vector<double> > auton_profile(1500, std::vector<double>(5)); //rows stacked on rows, all points // can't be in .h for some reason

double last_target_heading = 0.0;
double last_yaw_angle = 0.0;


double getSpeedFromTicksPer100Milliseconds(int ticks_per_second) {
	return TICKS_TO_DISTANCE * ticks_per_second * 10;
}

double getDistanceFromTicks(int ticks) {
	return TICKS_TO_DISTANCE * ticks;
}

//WestCoast, 2-speed transmission option
DriveBase::DriveBase(int l1, int l2,
		int r1, int r2, int pcm, int f_channel, int r_channel, bool two_speed) {

	k_p_yaw_au = K_P_YAW_AU; //these get sent from AutonDrive to Controller, not used in AutonDrive
	k_d_yaw_au = K_D_YAW_AU;

	max_vel_vis = MAX_VEL_VIS; //m/s for pathfinder
	max_acc_vis = MAX_ACC_VIS;
	max_jerk_vis = MAX_JERK_VIS;

	k_p_yaw_heading_pos = K_P_YAW_HEADING_POS;
	k_d_vision_pos = K_D_VISION_POS;

	k_p_yaw_dis = K_P_YAW_DIS;
	k_i_yaw_dis = K_I_YAW_DIS;
	k_d_yaw_dis = K_D_YAW_DIS;

	ff_scale = FF_SCALE;

	DYN_MAX_Y_RPM = max_y_rpm;

	if (two_speed) { //StartLow()

		// max_y_rpm = MAX_Y_RPM_LOW;
		// max_yaw_rate = MAX_YAW_RATE_LOW;
		// actual_max_y_rpm = ACTUAL_MAX_Y_RPM_LOW;
		// max_yaw_rate = (max_yaw_rate / actual_max_y_rpm) * max_y_rpm;
		//
		// k_p_right_vel = K_P_RIGHT_VEL_LOW;
		// k_p_left_vel = K_P_LEFT_VEL_LOW;
		// k_p_yaw_vel = K_P_YAW_VEL_LOW;
		// k_d_yaw_vel = K_D_YAW_VEL_LOW;
		// k_d_right_vel = K_D_RIGHT_VEL_LOW;
		// k_d_left_vel = K_D_LEFT_VEL_LOW;
		//
		// k_f_left_vel = 1.0 / actual_max_y_rpm;
		// k_f_right_vel = 1.0 / actual_max_y_rpm;
		//
		// is_low_gear = true;

	} else { // set drive variables to constants for regular drive controller

		max_y_rpm = MAX_Y_RPM;
		max_yaw_rate = MAX_YAW_RATE;
		actual_max_y_rpm = ACTUAL_MAX_Y_RPM;
		Kv = 1 / ACTUAL_MAX_Y_RPM;
		//max_yaw_rate = (max_yaw_rate / actual_max_y_rpm) * max_y_rpm;

		k_p_right_vel = K_P_RIGHT_VEL;
		k_p_left_vel = K_P_LEFT_VEL;
		k_p_yaw_vel = K_P_YAW_VEL;
		k_d_yaw_vel = K_D_YAW_VEL;
		k_d_right_vel = K_D_RIGHT_VEL;
		k_d_left_vel = K_D_LEFT_VEL;

		k_f_left_vel = 1.0 / actual_max_y_rpm;
		k_f_right_vel = 1.0 / actual_max_y_rpm;

	}

	// Place ACTUAL_MAX_RPM constants into local scope variables
  	actual_max_y_rpm_auton = ACTUAL_MAX_Y_RPM_AUTON;
  	actual_max_y_rpm_l_f = ACTUAL_MAX_Y_RPM_L_F;
  	actual_max_y_rpm_l_b = ACTUAL_MAX_Y_RPM_L_B;
  	actual_max_y_rpm_r_f = ACTUAL_MAX_Y_RPM_R_F;
  	actual_max_y_rpm_r_b = ACTUAL_MAX_Y_RPM_R_B;

	// Renames TalonFX Ids
	LF = l1;
	L2 = l2;
	RF = r1;
	R2 = r2;


	// Create TalonFXs
	canTalonLeft1 = new WPI_TalonFX(0);
	canTalonLeft2 = new WPI_TalonFX(0);
	canTalonRight1 = new WPI_TalonFX(0);
	canTalonRight2 = new WPI_TalonFX(0);
	
	canTalonLeft1->SetSafetyEnabled(false);
    canTalonLeft2->SetSafetyEnabled(false);
    canTalonRight1->SetSafetyEnabled(false);
    canTalonRight2->SetSafetyEnabled(false);
    canTalonLeft1->SetNeutralMode(NeutralMode::Brake);
    canTalonLeft2->SetNeutralMode(NeutralMode::Brake);
    canTalonRight1->SetNeutralMode(NeutralMode::Brake);
    canTalonRight2->SetNeutralMode(NeutralMode::Brake);

	
	// Configure front TalonFXs to use Integrated Encoders
	canTalonLeft1->ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor);
	canTalonRight1->ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor);

	canTalonRight1->SetInverted(true); // Invert Right Side TalonFX front
	canTalonRight2->SetInverted(true);

	// Configure back TalonFXs to follow their side's front TalonFX
 	canTalonLeft2->Follow(*canTalonLeft1);
  	canTalonRight2->Follow(*canTalonRight1);
	
	
	// Configure Current Limits
	canTalonLeft1->ConfigSupplyCurrentLimit(SupplyCurrentLimitConfiguration(true, 30, 30, 10));
	canTalonRight1->ConfigSupplyCurrentLimit(SupplyCurrentLimitConfiguration(true, 30, 30, 10));
 
	// Configure Ramp time to go from stop to full speed (in seconds)
	canTalonLeft1->ConfigOpenloopRamp(0.15, 0);
	canTalonLeft2->ConfigOpenloopRamp(0.15, 0);

	canTalonRight1->ConfigOpenloopRamp(0.15, 0);
	canTalonRight2->ConfigOpenloopRamp(0.15, 0);

	canTalonLeft1->ConfigVelocityMeasurementPeriod(
			VelocityMeasPeriod::Period_10Ms, 0);
	canTalonLeft1->ConfigVelocityMeasurementWindow(5, 0);

	canTalonRight1->ConfigVelocityMeasurementPeriod(
			VelocityMeasPeriod::Period_10Ms, 0);
	canTalonRight1->ConfigVelocityMeasurementWindow(5, 0);

	canTalonLeft1->SetControlFramePeriod(ControlFrame::Control_3_General, 5); //set talons every 5ms, default is 10
	canTalonLeft1->SetStatusFramePeriod(
			StatusFrameEnhanced::Status_2_Feedback0, 10, 0);

  canTalonRight1->SetControlFramePeriod(ControlFrame::Control_3_General, 5); //set talons every 5ms, default is 10
  canTalonRight1->SetStatusFramePeriod(
    			StatusFrameEnhanced::Status_2_Feedback0, 10, 0);

  canTalonLeft1->ConfigVoltageCompSaturation(12.0);
  canTalonLeft1->EnableVoltageCompensation(true);
  canTalonRight1->ConfigVoltageCompSaturation(12.0);
  canTalonRight1->EnableVoltageCompensation(true);

  canTalonLeft2->ConfigVoltageCompSaturation(12.0);
  canTalonLeft2->EnableVoltageCompensation(true);
  canTalonRight2->ConfigVoltageCompSaturation(12.0);
  canTalonRight2->EnableVoltageCompensation(true);

  ahrs = new AHRS(SerialPort::kUSB);
}


void DriveBase::ResetConfigs() {
		canTalonLeft1->SetSafetyEnabled(false);
    canTalonLeft2->SetSafetyEnabled(false);
    canTalonRight1->SetSafetyEnabled(false);
    canTalonRight2->SetSafetyEnabled(false);
    canTalonLeft1->SetNeutralMode(NeutralMode::Brake);
    canTalonLeft2->SetNeutralMode(NeutralMode::Brake);
    canTalonRight1->SetNeutralMode(NeutralMode::Brake);
    canTalonRight2->SetNeutralMode(NeutralMode::Brake);

	
	// Configure front TalonFXs to use Integrated Encoders
	canTalonLeft1->ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor);
	canTalonRight1->ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor);

	canTalonRight1->SetInverted(true); // Invert Right Side TalonFX front
	canTalonRight2->SetInverted(true);

	// Configure back TalonFXs to follow their side's front TalonFX
 	canTalonLeft2->Follow(*canTalonLeft1);
  	canTalonRight2->Follow(*canTalonRight1);
	
	
	// Configure Current Limits
	canTalonLeft1->ConfigSupplyCurrentLimit(SupplyCurrentLimitConfiguration(true, 30, 30, 10));
	canTalonRight1->ConfigSupplyCurrentLimit(SupplyCurrentLimitConfiguration(true, 30, 30, 10));
 
	// Configure Ramp time to go from stop to full speed (in seconds)
	canTalonLeft1->ConfigOpenloopRamp(0.15, 0);
	canTalonLeft2->ConfigOpenloopRamp(0.15, 0);

	canTalonRight1->ConfigOpenloopRamp(0.15, 0);
	canTalonRight2->ConfigOpenloopRamp(0.15, 0);

	canTalonLeft1->ConfigVelocityMeasurementPeriod(
			VelocityMeasPeriod::Period_10Ms, 0);
	canTalonLeft1->ConfigVelocityMeasurementWindow(5, 0);

	canTalonRight1->ConfigVelocityMeasurementPeriod(
			VelocityMeasPeriod::Period_10Ms, 0);
	canTalonRight1->ConfigVelocityMeasurementWindow(5, 0);

	canTalonLeft1->SetControlFramePeriod(ControlFrame::Control_3_General, 5); //set talons every 5ms, default is 10
	canTalonLeft1->SetStatusFramePeriod(
			StatusFrameEnhanced::Status_2_Feedback0, 10, 0);

  	canTalonRight1->SetControlFramePeriod(ControlFrame::Control_3_General, 5); //set talons every 5ms, default is 10
  	canTalonRight1->SetStatusFramePeriod(
    			StatusFrameEnhanced::Status_2_Feedback0, 10, 0);

	canTalonLeft1->ConfigVoltageCompSaturation(12.0);
  	canTalonLeft1->EnableVoltageCompensation(true);
  	canTalonRight1->ConfigVoltageCompSaturation(12.0);
  	canTalonRight1->EnableVoltageCompensation(true);

  	canTalonLeft2->ConfigVoltageCompSaturation(12.0);
	canTalonLeft2->EnableVoltageCompensation(true);
  	canTalonRight2->ConfigVoltageCompSaturation(12.0);
  	canTalonRight2->EnableVoltageCompensation(true);
}


double max_fwd_speed_l = 0;
double max_fwd_speed_r = 0;
double max_yaw_rate_ = 0;

void DriveBase::ManualOpenLoopDrive(Joystick* throttle, Joystick* wheel) {
	double throttle_val = throttle->GetY();
	double wheel_val = wheel->GetX();

	double left_out, right_out;

	left_out = throttle_val - wheel_val / 2.0f;
	right_out = throttle_val + wheel_val / 2.0f;

	canTalonLeft1->Set(ControlMode::PercentOutput, left_out);
	canTalonRight1->Set(ControlMode::PercentOutput, left_out);

	double cspeed_l = abs(getSpeedFromTicksPer100Milliseconds(canTalonLeft1->GetSelectedSensorVelocity())) ;
	double cspeed_r = abs(getSpeedFromTicksPer100Milliseconds(canTalonRight1->GetSelectedSensorVelocity()));

	if (cspeed_l > max_fwd_speed_l) {
		max_fwd_speed_l = cspeed_l;
	}

	if (cspeed_r > max_fwd_speed_r) {
		max_fwd_speed_r = cspeed_r;
	}

	double cyawrate = ahrs->GetRate();

	if (cyawrate > max_yaw_rate) {
		max_yaw_rate = cyawrate;
	}


	frc::SmartDashboard::PutNumber("max fwd speed left", max_fwd_speed_l);
	frc::SmartDashboard::PutNumber("max fwd speed right", max_fwd_speed_r);
	frc::SmartDashboard::PutNumber("fwd speed left", cspeed_l);
	frc::SmartDashboard::PutNumber("fwd speed right", cspeed_r);

	frc::SmartDashboard::PutNumber("yaw rate", cyawrate);
	frc::SmartDashboard::PutNumber("max yaw rate", max_yaw_rate);

}



//PD on left and right
//P on yaw
void DriveBase::TeleopWCDrive(Joystick *JoyThrottle, //finds targets for the Controller()
		Joystick *JoyWheel, bool pos_yaw, bool vis_yaw) {

	//		led_solenoid->Set(true);

	double target_l, target_r, target_yaw_rate;

	double throttle = JoyThrottle->GetY();

	double reverse_y = 1.0;

	if (throttle > 0.0) {
		reverse_y = -1.0;
	} else {
		reverse_y = 1.0;
	}

	double forward = (throttle) * (throttle); //squared and will always be positive, so we need reverse_y

	target_l = reverse_y * forward * max_y_rpm; //scale to velocity

	target_r = target_l;

	if (pos_yaw) { //pos control wheel
	///	led_solenoid->Set(false);

			double target_heading = init_heading + (-1.0 * JoyWheel->GetX() * (90.0 * apes::PI / 180.0));
		frc::SmartDashboard::PutNumber("init head", init_heading);
			frc::SmartDashboard::PutNumber("targ head", target_heading);
		//frc::SmartDashboard::PutNumber("targ yaw to", visionDrive->GetYawToTarget());

			double current_heading = -1.0 * ahrs->GetYaw() * ( apes::PI / 180.0); //degrees to radians, left should be positive
		frc::SmartDashboard::PutNumber("cur head", current_heading);
			double error_heading = target_heading - current_heading;
		frc::SmartDashboard::PutNumber("error head", error_heading);
			target_yaw_rate = 1.0 * 0.1 * error_heading * max_yaw_rate;
			frc::SmartDashboard::PutNumber("targ RATE", target_yaw_rate); //fine

			k_p_yaw_vel = 10.0;
			k_p_left_vel = K_P_RIGHT_VEL;
			k_p_right_vel = K_P_LEFT_VEL;

} else if (vis_yaw) { //pos control vision
	//	led_solenoid->Set(true);
	// k_p_left_vel = 0.015;
	// k_p_right_vel = 0.015;
	// k_p_yaw_vel = 20.0;

    double current_heading = -1.0 * ahrs->GetYaw() * ( apes::PI / 180.0); //degrees to radians, left should be positive
		double yaw_angle = 0.0f;
		double target_heading = last_target_heading;

		// if the yaw angle has changed, recalculate the target heading, otherwise keep the same one
		if (yaw_angle != last_yaw_angle) { //may need to set threshold
			target_heading = current_heading - yaw_angle;
		}

		frc::SmartDashboard::PutNumber("init head", init_heading);
    frc::SmartDashboard::PutNumber("targ head", target_heading);
  	frc::SmartDashboard::PutNumber("targ yaw to", yaw_angle);

		frc::SmartDashboard::PutNumber("cur head", current_heading);
    double error_heading = target_heading - current_heading;
		frc::SmartDashboard::PutNumber("error head", error_heading);
    target_yaw_rate = 1.0  * 0.20 * error_heading * max_yaw_rate;
    frc::SmartDashboard::PutNumber("targ RATE", target_yaw_rate); //fine

		last_yaw_angle = yaw_angle;
		last_target_heading = target_heading;

} else { //vel control wheel
//	led_solenoid->Set(false);
	double reverse_x = 1.0;
	double wheel = -1.0 * JoyWheel->GetX();

	if (wheel < 0.0) {
		reverse_x = 1.0;//for black wheel, is opposite
	} else {
		reverse_x = -1.0;
	}

	double joy_wheel_val = reverse_x * wheel * wheel;

	if (std::abs(joy_wheel_val) < .02) {
		joy_wheel_val = 0.0;
	}

	target_yaw_rate = -1.0 * (joy_wheel_val) * max_yaw_rate; //Left will be positive

	k_p_left_vel = K_P_LEFT_VEL;
	k_p_right_vel = K_P_RIGHT_VEL;
	k_p_yaw_vel = K_P_YAW_VEL;

}

	if (target_l > max_y_rpm) {
		target_l = max_y_rpm;
	} else if (target_l < -max_y_rpm) {
		target_l = -max_y_rpm;
	}

	if (target_r > max_y_rpm) {
		target_r = max_y_rpm;
	} else if (target_r < -max_y_rpm) {
		target_r = -max_y_rpm;
	}

	frc::SmartDashboard::PutNumber("target_r", target_r);
	frc::SmartDashboard::PutNumber("target_l", target_l);
	
	double cyawrate = ahrs->GetRate();

	if (cyawrate > max_yaw_rate_) {
		max_yaw_rate_ = cyawrate;
	}

		frc::SmartDashboard::PutNumber("yaw rate", cyawrate);
	frc::SmartDashboard::PutNumber("max yaw rate", max_yaw_rate_);
	// last_speed = GetForwardSpeed();
	// last_angular_speed = GetAngularSpeed();
	Controller(0.0, target_r, target_l, target_yaw_rate, k_p_right_vel,
			k_p_left_vel, 0.0, k_p_yaw_vel, 0.0, k_d_right_vel, k_d_left_vel, 0.0,
			0.0, 0.0, 0.0);

}



void DriveBase::RotationController(Joystick *JoyWheel) {
//	frc::SmartDashboard::PutNumber("joyWheel", JoyWheel->GetX());

	double target_heading = init_heading
			+ (-1.0 * JoyWheel->GetX() * (90.0 * apes::PI / 180.0)); //scaling, conversion to radians,left should be positive

	double current_heading = -1.0 * ahrs->GetYaw() * ( apes::PI / 180.0); //degrees to radians, left should be positive

//	frc::SmartDashboard::PutNumber("current heading", current_heading);
//	frc::SmartDashboard::PutNumber("target heading", target_heading);

	double error_heading_h = target_heading - current_heading;
	double total_heading_h = k_p_yaw_heading_pos + error_heading_h;

	//frc::SmartDashboard::PutNumber("total heading", total_heading);
	//frc::SmartDashboard::PutNumber("k_p_yaw_heading_pos", k_p_yaw_heading_pos);

	if (total_heading > max_yaw_rate) {
		total_heading = max_yaw_rate;
	} else if (total_heading < -max_yaw_rate) {
		total_heading = -max_yaw_rate;
	}

	double k_p_yaw_h_vel = 10.0;

	Controller(0.0, 0.0, 0.0, total_heading_h, k_p_right_vel, k_p_left_vel,
			0.0, k_p_yaw_h_vel, 0.0, k_d_right_vel, k_d_left_vel,
			0.0, 0.0, 0.0, 0.0);

}

void DriveBase::Controller(double ref_kick,
		double ref_right, //first parameter refs are for teleop
		double ref_left, double ref_yaw, double k_p_right, double k_p_left,
		double k_p_kick, double k_p_yaw, double k_d_yaw, double k_d_right,
		double k_d_left, double k_d_kick, double target_vel_left, //50
		double target_vel_right, double target_vel_kick) { //last parameter targets are for auton

	double yaw_rate_current = -1.0 * (double) ahrs->GetRate(); //might be rad/ss
		//	* (double) ((apes::PI) / 180.0); //left should be positive

	 // frc::SmartDashboard::PutNumber("yaw vel", yaw_rate_current);
	 frc::SmartDashboard::PutNumber("yaw pos", ahrs->GetYaw());
	/// frc::SmartDashboard::PutNumber("max_y_rpm", max_y_rpm);
	// frc::SmartDashboard::PutNumber("max_yaw_rate", max_yaw_rate);

	double target_yaw_rate = ref_yaw;

	ref_left = ref_left - (target_yaw_rate * (max_y_rpm / max_yaw_rate)); //left should be positive
	ref_right = ref_right + (target_yaw_rate * (max_y_rpm / max_yaw_rate)); //ff

	//ref_left = ref_left + (target_yaw_rate * (max_y_rpm / max_yaw_rate)); //left should be positive
	//ref_right = ref_right - (target_yaw_rate * (max_y_rpm / max_yaw_rate)); //ff

	double yaw_error = target_yaw_rate - yaw_rate_current;

	frc::SmartDashboard::PutNumber("yaw vel error", yaw_error);
	frc::SmartDashboard::PutNumber("yaw raet target", target_yaw_rate);
	frc::SmartDashboard::PutNumber("yaw currant", yaw_rate_current);

//	if(yaw_rate_current == 0.0) {
//		k_p_yaw = 0.0;
//		k_d_yaw = 0.0;
//	}

	if (std::abs(yaw_error) < .3) { //TODO: maybe get rid of this
		yaw_error = 0.0;
	}

	d_yaw_dis = yaw_error - yaw_last_error;

	double yaw_output = ((k_p_yaw * yaw_error) + (k_d_yaw * d_yaw_dis)); //pd for auton, p for teleop //fb //hardly any
//frc::SmartDashboard::PutNumber("yaw p", yaw_output);
	ref_right += yaw_output; //left should be positive
	ref_left -= yaw_output;

	if (std::abs(ref_kick) < 25) {
		ref_kick = 0;
	}

	if (ref_left > max_y_rpm) {
		ref_left = max_y_rpm;
	} else if (ref_left < -max_y_rpm) {
		ref_left = -max_y_rpm;
	}

	if (ref_right > max_y_rpm) {
		ref_right = max_y_rpm;
	} else if (ref_right < -max_y_rpm) {
		ref_right = -max_y_rpm;
	}

  if (frc::RobotState::IsAutonomous()) { //only want the feedforward based off the motion profile during autonomous. The root generated ones (in the if() statement) //should already be 0 during auton because we send 0 as refs
    feed_forward_r = 0;	// will be close to 0  (low error between profile points) for the most part but will get quite aggressive when an error builds,
    feed_forward_l = 0;			//the PD controller should handle it itself
    feed_forward_k = 0;

  } else {

    if (ref_right < 0.0) {
      k_f_right_vel = 1.0 / actual_max_y_rpm_r_b;
    } else {
      k_f_right_vel = 1.0 / actual_max_y_rpm_r_f;
    }

    if (ref_left < 0.0) {
      k_f_left_vel = 1.0 / actual_max_y_rpm_l_b;
    } else {
      k_f_left_vel = 1.0 / actual_max_y_rpm_l_f;
    }

    feed_forward_r = k_f_right_vel * ref_right; //teleop only, controlled
  	feed_forward_l = k_f_left_vel * ref_left;
  	feed_forward_k = 0.0 * ref_kick;//kf kick vel
  }


	// frc::SmartDashboard::PutNumber("kf r", k_f_right_vel);
	// frc::SmartDashboard::PutNumber("kf l", k_f_left_vel);
	//
	// frc::SmartDashboard::PutNumber("ff r", feed_forward_r * MAX_Y_RPM); //max rpm
	// frc::SmartDashboard::PutNumber("ff l", feed_forward_l * MAX_Y_RPM);

	//conversion to RPM from native unit
	double l_current = GetLeftVel(); //-((double) canTalonLeft1->GetSelectedSensorVelocity(0)
	//		/ (double) TICKS_PER_ROT) * MINUTE_CONVERSION;
	double r_current = GetRightVel(); //((double) canTalonRight1->GetSelectedSensorVelocity(0)
	//		/ (double) TICKS_PER_ROT) * MINUTE_CONVERSION;
//	double kick_current = ((double) canTalonKicker->GetSelectedSensorVelocity(0) //will timeout, taking too much time
//			 (double) TICKS_PER_ROT) * MINUTE_CONVERSION; //going right is positive

// frc::SmartDashboard::PutNumber("l position", GetLeftPosition());
// frc::SmartDashboard::PutNumber("r position", GetRightPosition());

	l_error_vel_t = ref_left - l_current;
	r_error_vel_t = ref_right - r_current;
	//kick_error_vel = ref_kick - kick_current;


  frc::SmartDashboard::PutNumber("l current", l_current);
  frc::SmartDashboard::PutNumber("r current", r_current);
	//
  // 	frc::SmartDashboard::PutNumber("l vel targ", ref_left);
  // 	frc::SmartDashboard::PutNumber("r vel targ", ref_right);

	d_left_vel = (l_error_vel_t - l_last_error_vel);
	d_right_vel = (r_error_vel_t - r_last_error_vel);
	d_kick_vel = (kick_error_vel - kick_last_error_vel);

  frc::SmartDashboard::PutNumber("l vel error", l_error_vel_t);
  frc::SmartDashboard::PutNumber("r vel error", r_error_vel_t);
	//
  // frc::SmartDashboard::PutNumber("l vel k", k_p_left);
  // frc::SmartDashboard::PutNumber("r vel k", k_p_right);
	//
	// frc::SmartDashboard::PutNumber("k p l", k_p_left);
	//   frc::SmartDashboard::PutNumber("k p r", k_p_right);
	//  frc::SmartDashboard::PutNumber("R2",canTalonRight2->GetOutputCurrent());

	P_LEFT_VEL = k_p_left * l_error_vel_t;
	P_RIGHT_VEL = k_p_right * r_error_vel_t;
	P_KICK_VEL = k_p_kick * kick_error_vel;

	D_LEFT_VEL = k_d_left * d_left_vel;
	D_RIGHT_VEL = k_d_right * d_right_vel;
	D_KICK_VEL = k_d_kick * d_kick_vel;

   // frc::SmartDashboard::PutNumber("L2", drive_controller->canTalonLeft2->GetOutputCurrent());
  ///frc::SmartDashboard::PutNumber("R1", canTalonRight1->GetOutputCurrent());
  // //  frc::SmartDashboard::PutNumber("R1", drive_controller->GetRightVel());
  //  frc::SmartDashboard::PutNumber("R2",canTalonRight2->GetOutputCurrent());
  //  frc::SmartDashboard::PutNumber("R3", canTalonRight3->GetOutputCurrent());
  //
  //  frc::SmartDashboard::PutNumber("L1", canTalonLeft1->GetOutputCurrent());
  // //  frc::SmartDashboard::PutNumber("R1", drive_controller->GetRightVel());
  //  frc::SmartDashboard::PutNumber("L2", canTalonLeft2->GetOutputCurrent());
  //  frc::SmartDashboard::PutNumber("L3", canTalonLeft3->GetOutputCurrent());

  // frc::SmartDashboard::PutNumber("D r Vel", D_RIGHT_VEL *550.0);
  // frc::SmartDashboard::PutNumber("P r Vel", P_RIGHT_VEL*550.0);
  // frc::SmartDashboard::PutNumber("D l Vel", D_LEFT_VEL*550.0);
  // frc::SmartDashboard::PutNumber("P l Vel", P_LEFT_VEL*550.0);

	frc::SmartDashboard::PutNumber("D_RIGHT_VEL", D_RIGHT_VEL);
	frc::SmartDashboard::PutNumber("P_RIGHT_VEL", P_RIGHT_VEL);
	frc::SmartDashboard::PutNumber("feed_forward_r", feed_forward_r);

	double total_right = D_RIGHT_VEL + P_RIGHT_VEL + feed_forward_r
			+ (Kv * target_vel_right * ff_scale); //Kv only in auton, straight from motion profile
	double total_left = D_LEFT_VEL + P_LEFT_VEL + feed_forward_l
			+ (Kv * target_vel_left * ff_scale);
//	double total_kick = D_KICK_VEL + P_KICK_VEL + feed_forward_k
//			+ (Kv_KICK * target_vel_kick);

	if (total_right > 1.0) {
		total_right = 1.0;
	} else if (total_right < -1.0) {
		total_right = -1.0;
	}
	if (total_left > 1.0) {
		total_left = 1.0;
	} else if (total_left < -1.0) {
		total_left = -1.0;
	}

	frc::SmartDashboard::PutNumber("% OUT LEFT", total_left);
	frc::SmartDashboard::PutNumber("% OUT RIGHT", total_right);

  canTalonLeft1->Set(ControlMode::PercentOutput, total_left);
	canTalonRight1->Set(ControlMode::PercentOutput, total_right);


	

	double cspeed_l = getSpeedFromTicksPer100Milliseconds(canTalonLeft1->GetSelectedSensorVelocity());
	double cspeed_r = getSpeedFromTicksPer100Milliseconds(canTalonRight1->GetSelectedSensorVelocity());
	frc::SmartDashboard::PutNumber("fwd speed left", cspeed_l);
	frc::SmartDashboard::PutNumber("fwd speed right", cspeed_r);


	// canTalonRight2->Set(ControlMode::PercentOutput, -total_right);

	yaw_last_error = yaw_error;
	l_last_error_vel = l_error_vel_t;
	r_last_error_vel = r_error_vel_t;
	kick_last_error_vel = kick_error_vel;
	l_last_current = l_current;


}

void DriveBase::ZeroAll(bool stop_motors) {

	if (stop_motors) {
		StopAll();
	}

	ZeroI();
	ZeroEncs();
	ZeroYaw();

	zeroing_counter++;

}

//will stop all driven motors in the drive controller
void DriveBase::StopAll() {

	canTalonLeft1->Set(ControlMode::PercentOutput, 0.0);
	canTalonRight1->Set(ControlMode::PercentOutput, 0.0);

}

//sets the position of all the drive encoders to 0
void DriveBase::ZeroEncs() {

	canTalonRight1->SetSelectedSensorPosition(0, 0, 0);
	canTalonLeft1->SetSelectedSensorPosition(0, 0, 0);

}

void DriveBase::ZeroYaw() {

	ahrs->ZeroYaw();

}

double DriveBase::GetLeftVel() { //550 left back //590 left forward

	double l_current = ((double) canTalonLeft1->GetSelectedSensorVelocity(0)
			/ (double) TICKS_PER_ROT) * MINUTE_CONVERSION;

	return l_current;
}

double DriveBase::GetRightVel() { //

	double r_current = ((double) canTalonRight1->GetSelectedSensorVelocity(0)
			/ (double) TICKS_PER_ROT) * MINUTE_CONVERSION;

	return r_current;
}

double DriveBase::GetYawPos() {

	double y_dis = -1.0 * ahrs->GetYaw() * (double) (apes::PI / 180);
	return y_dis;

}

//Zeros the accumulating I
void DriveBase::ZeroI() {

	i_right = 0;
	i_left = 0;
	i_yaw = 0;

	y_error_dis_au = 0;
	l_error_dis_au = 0;
	r_error_dis_au = 0;

}


void DriveBase::SetMaxRpm(double rpm) {

	max_y_rpm = rpm;

}

double DriveBase::GetMaxRpm() {

	return max_y_rpm;

}


double DriveBase::GetLeftPosition() {

	double l_dis = -1.0 * ((double) canTalonLeft1->GetSelectedSensorPosition(0)
			/ TICKS_PER_FOOT);

	return l_dis;

}

double DriveBase::GetRightPosition() {

	double r_dis = -1.0 * ((double) canTalonRight1->GetSelectedSensorPosition(0)
			/ TICKS_PER_FOOT);

	return r_dis;

}

//returns rad
double DriveBase::GetRoll() {

	return (ahrs->GetRoll() * 3.14159 / 180.0);

}

bool DriveBase::IsLastIndex() {

	return is_last_index;

}

int DriveBase::GetDriveIndex() {

	return row_index;

}

void DriveBase::SetZeroingIndex(std::vector<int> zero_index) {

	zeroing_index = zero_index;

}


//Increments through target points of the motion profile
//Pre: SetAutonRefs()
//		row_index = 0, vision_profile filled, zeroing_indeces filled if needed, zero_counter = 0 if needed, continue_profile set as needed


void DriveBase::RunTeleopDrive(Joystick *JoyThrottle,
	Joystick *JoyWheel, bool is_regular, bool is_vision, bool is_rotation) {
//let go of rot button, force vel to 0 until wheel goes to 0
		 if (is_rotation) {
			teleop_drive_state = ROTATION_CONTROLLER;
		} if (is_vision) {
			teleop_drive_state = VISION_DRIVE;
		}

		switch (teleop_drive_state) {

			case REGULAR:
			frc::SmartDashboard::PutString("DRIVE", "reg");
			TeleopWCDrive(JoyThrottle, JoyWheel, false, false);
			last_drive_state = REGULAR;
			break;
			case ROTATION_CONTROLLER:
			frc::SmartDashboard::PutString("DRIVE", "rot");
      if (last_drive_state != ROTATION_CONTROLLER) {
				 init_heading = -1.0 * ahrs->GetYaw() * 3.14 / 180.0; //stamp
			}
			if (!is_rotation) { //for transition back to vel wheel control
				if (JoyWheel->GetX() < 0.1) {
					teleop_drive_state = REGULAR;
				}
			} else {
			TeleopWCDrive(JoyThrottle, JoyWheel, true, false);
			}
			last_drive_state = ROTATION_CONTROLLER;
			break;
		}

}



// double DriveBase::GetAngularSpeed() {
// 	return DEG2RAD(ahrs->GetRate());
// }
// double DriveBase::GetAngularAcceleration() {
// 	return (GetAngularSpeed() - last_angular_speed) / seconds_since_last_update;
// }
// double DriveBase::GetForwardSpeed() {
// 	return (GetLeftVel() + GetRightVel()) / 2;
// }
// double DriveBase::GetForwardAcceleration() {
// 	return (GetForwardSpeed() - last_speed) / seconds_since_last_update;
// };