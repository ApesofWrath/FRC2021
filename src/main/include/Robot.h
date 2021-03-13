/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>

#include <frc/Joystick.h>
#include "ControlPanel.h"
#include "Drive/DriveController.h"
#include "RobotContainer.h"
#include "Auton/AutonDrive.h"
#include "AHRS.h"

#include "Shooter.h"
#include "Arm.h"
#include "Intake.h"
#include <cameraserver/CameraServer.h>


#include "rev/ColorSensorV3.h"

#include "TeleopStateMachine.h"

#define BUTTON_STOP_INTAKE     0
#define BUTTON_IN_INTAKE       0
#define BUTTON_OUT_INTAKE      0
#define BUTTON_STOP_SHOOTER    0
#define BUTTON_INTAKE_SHOOTER  0
#define BUTTON_SHOOT_SHOOTER   0
#define BUTTON_WAITING_SHOOTER 0
#define BUTTON_WFB             13


class Robot : public frc::TimedRobot {
 public: 

  ControlPanel* controlpanel; 

  DriveController* drive;

//Temporary Button ids
  const int BUTTON_STOP = 2, POSITION_BUTTON = 5, ROTATION_BUTTON = 4, INTAKE = 3;

  Colors currentColor, desiredColor;
  frc::Joystick  *JoyThrottle, *JoyWheel, *JoyOp;

  Arm* arm;
  Intake* intake;
  // TalonSRX* talon0;
  Shooter* shooter, speed;

  TeleopStateMachine *tsm;

  rev::CANSparkMax *neo_1, *neo_2, *neo_3, *neo_4;
  const double CONTROL_WHEEL_SPEED_ON = 1.0f;
  const double CONTROL_WHEEL_SPEED_OFF = 0;

  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;
  std::string getColor(Colors c);
    
  void UpdateButtons();

  bool stop_intake, in_intake, out_intake;
  
  bool shoot_shooter, intake_shooter, stop_shooter, waiting_shooter;

 private:
  const Colors kDesColorDefault = Colors::WHITE;
  // const std::string kAutoNameDefault = "Default";
  // const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;
  AutonDrive *a_drive;
  frc::SendableChooser<Colors> m_descolor_chooser;
  
  frc2::Command* m_autonomousCommand = nullptr;
  RobotContainer *m_container;
};
