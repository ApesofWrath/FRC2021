#pragma once

#include <units/units.h>
#include "Constants.h"
#include <units/units.h>

#define PRACTICE 0 //0 uses the consts on the bottom //comp gains work ok on practice bot too

#define PI 3.1415926535897932

// #define SWERVE_DRIVE


#ifndef SWERVE_DRIVE
#if PRACTICE //bot still w/o superstructure

//Teleop

const double reverse_output = -1.0;

const double K_P_RIGHT_VEL = 0.003;//0.0001; //no gear shift
const double K_D_RIGHT_VEL = 0.0;// 0.0005;

const double K_P_LEFT_VEL = 0.003; //voltage compensation //ff //p
const double K_D_LEFT_VEL = 0.0;

const double K_P_YAW_VEL = 13.0;
const double K_D_YAW_VEL = 0.0;

const double K_P_YAW_HEADING_POS = 0.0; //special controllers - yaw vision
const double K_D_VISION_POS = 0.0;

//Auton

const double K_P_YAW_AU = 0.0; //gets sent to Controller()
const double K_D_YAW_AU = 0.0;

const double K_P_YAW_DIS = 0.0; //used in AutonDrive()
const double K_I_YAW_DIS = 0.0;
const double K_D_YAW_DIS = 0.0;

const double K_P_RIGHT_DIS = 0.0;
const double K_I_RIGHT_DIS = 0.0;
const double K_D_RIGHT_DIS = 0.0;

const double K_P_LEFT_DIS = 0.0;
const double K_I_LEFT_DIS = 0.0;
const double K_D_LEFT_DIS = 0.0;

const double MAX_VEL_VIS = 3.0; //m/s for pathfinder
const double MAX_ACC_VIS = 1.0;
const double MAX_JERK_VIS = 10000.0;

//Drive maxes

const double ACTUAL_MAX_Y_RPM_AUTON = 10.0;
const double ACTUAL_MAX_Y_RPM_L_F = 600.0;
const double ACTUAL_MAX_Y_RPM_L_B = 595.0;
const double ACTUAL_MAX_Y_RPM_R_F = 590.0;
const double ACTUAL_MAX_Y_RPM_R_B = 595.0;

const double MAX_Y_RPM = 595.0; //smallest actual max
const double ACTUAL_MAX_Y_RPM = 600.0;
const double MAX_YAW_RATE = 11.4;
const double MAX_FPS = 15.0; //used in auton pathfinder

const double FF_SCALE = 0.7; //auton

#else //1st bot to have superstructure

//Teleop

const double reverse_output = 1.0;

const double K_P_RIGHT_VEL = 0.001558;//0.003;//0.0001; //no gear shift
const double K_D_RIGHT_VEL = 0.0;// 0.0005;

const double K_P_LEFT_VEL = 0.00103;//0.003; //voltage compensation //ff //p
const double K_D_LEFT_VEL = 0.0;

const double K_P_YAW_VEL = 0.8;//13.0;
const double K_D_YAW_VEL = 0.0;

const double K_P_YAW_HEADING_POS = 0.0; //special controllers - yaw vision
const double K_D_VISION_POS = 0.0;

//Auton

const double K_P_YAW_AU = 0.0; //gets sent to Controller()
const double K_D_YAW_AU = 0.0;

const double K_P_YAW_DIS = 0.0; //used in AutonDrive()
const double K_I_YAW_DIS = 0.0;
const double K_D_YAW_DIS = 0.0;

const double K_P_RIGHT_DIS = 0.0;
const double K_I_RIGHT_DIS = 0.0;
const double K_D_RIGHT_DIS = 0.0;

const double K_P_LEFT_DIS = 0.0;
const double K_I_LEFT_DIS = 0.0;
const double K_D_LEFT_DIS = 0.0;

const double MAX_VEL_VIS = 3.0; //m/s for pathfinder
const double MAX_ACC_VIS = 1.0;
const double MAX_JERK_VIS = 10000.0;

//Drive maxes


const double MAX_FORWARD_MPS = 4.89154;

const double MAX_Y_RPM = MAX_FORWARD_MPS / ((2 * PI * 3) * 0.0254) * 60;
const double ACTUAL_MAX_Y_RPM = MAX_Y_RPM;

const double ACTUAL_MAX_Y_RPM_AUTON = ACTUAL_MAX_Y_RPM;
const double ACTUAL_MAX_Y_RPM_L_F = ACTUAL_MAX_Y_RPM;
const double ACTUAL_MAX_Y_RPM_L_B = ACTUAL_MAX_Y_RPM;
const double ACTUAL_MAX_Y_RPM_R_F = ACTUAL_MAX_Y_RPM;
const double ACTUAL_MAX_Y_RPM_R_B = ACTUAL_MAX_Y_RPM;


const double MAX_YAW_RATE = 10.54;
const double MAX_FPS = 10.0;//; //used in auton pathfinder

const double FF_SCALE = 0.7; //auton

constexpr auto K_S = 0.105_V;
constexpr auto K_V = 2.39 * 1_V * 1_s / 1_m;
constexpr auto K_A = 0.202 * 1_V * 1_s * 1_s / 1_m;
constexpr auto K_R_SQUARED = 0.999_m;
constexpr auto K_TRACK_WIDTH = 0.7101365226646625;
constexpr auto K_P_AUTON_FB = 2.79;
constexpr auto K_D_AUTON_FB = 1.24;
constexpr auto K_MAX_ACCEL = 10.77;

#endif //both bots

// const double WHEEL_DIAMETER = 6.0; //inches, for fps for auton
// const double TICKS_PER_ROT = 2048 * ();//1365.0; //about 3 encoder rotations for each actual rotation // 4096 ticks per rotation for mag encoders
// const double TICKS_PER_FOOT = 1315.0; //auton
// const double MINUTE_CONVERSION = 600.0; //part of the conversion from ticks velocity to rpm

const double TICKS_PER_ROTATION_FALCON = 2048;
const double SWERVE_MOD_ROTATE_RATIO = 13.64;
const double SWERVE_MOD_DRIVE_RATIO = 7.39;
const double WHEEL_RADIUS = 0.0762;
const double METER_TO_INCHES = 39.3701;
const double WHEEL_CIRCUMFERENCE = 4 * PI;
const double METERS_PER_WHEEL_ROTATION = WHEEL_RADIUS * 2 * PI;
const double ANGLE_TO_TICKS_SWERVE_YAW = (TICKS_PER_ROTATION_FALCON * SWERVE_MOD_ROTATE_RATIO) / (2 * PI);
const double FALCON_MS_PER_UPDATE = 100.0;
const double MS_PER_SECOND = 1000.0;

const double FALCON_UPDATE_PER_SECOND = FALCON_MS_PER_UPDATE / MS_PER_SECOND;


struct PIDSettings {
    double kP = 0, kD = 0, kI = 0;
};

struct SwerveDrivePIDConfig {
    PIDSettings kFrontLeftDrive, kFrontLeftYaw;
    PIDSettings kFrontRightDrive, kFrontRightYaw;
    PIDSettings kBackLeftDrive, kBackLeftYaw;
    PIDSettings kBackRightDrive, kBackRightYaw;
};  


const SwerveDrivePIDConfig SWERVE_CONFIG_PID = {
    {0.005, 0, 0}, {0.005, 0, 0.0}, // Front Left
    {0.005, 0, 0}, {0.005, 0, 0.0}, // Front Right
    {0.005, 0, 0}, {0.005, 0, 0.0}, // Back Left
    {0.005, 0, 0}, {0.005, 0, 0.0}  // Back Right
};


#else // Swerve Drive


#endif
