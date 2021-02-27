#ifndef SRC_ARM_H_
#define SRC_ARM_H_

#include <ctre/Phoenix.h>
// #include <frc/WPILib.h>
#include "rev/CANSparkMax.h"
#include <frc/Joystick.h>

#include "Drive/DriveBase.h"
#include "Macros.h"

#define SMART_MODE_THRESHOLD_FWD_SPEED 0.1
#define SMART_MODE_THRESHOLD_FWD_ACCEL 0.1
#define SMART_MODE_THRESHOLD_ANG_SPEED 0.1
#define SMART_MODE_THRESHOLD_ANG_ACCEL 0

#define UP_POSITION 0.1994662002
#define DOWN_POSITION 1.26561304

class Arm {
public:

  // TalonSRX *talonArm;
  rev::CANEncoder *armEncoder;
  rev::CANPIDController *armPID;
  rev::CANAnalog *analog;
  rev::CANSparkMax *armSparkM0;
  rev::CANSparkMax *armSparkM1;

  frc::Joystick* joy;

  float armStartPos;
  float armCurrPos;
  float targetPos;

  // float upPos = 2;

  enum States {
    REST_STATE, UP_STATE, DOWN_STATE
  };

  States intake_arm_state = REST_STATE, last_intake_arm_state = REST_STATE;

  Arm();

  const int rightArmSpark = 21; // leader
  const int leftArmSpark = 24; //follower

  const float START_OFFSET = (UP_POSITION/(2*PI))*63;
  const float DOWN_POS = (DOWN_POSITION/(2*PI))*63;

  void Up();
  void Down();
  void Rest();
  void MoveToPosition(double desiredPosition);

  // void ShouldBeInSmartMode();

  void IntakeArmStateMachine();

  void UpperSoftLimit();
  void LowerSoftLimit();
 
  
};

#endif
