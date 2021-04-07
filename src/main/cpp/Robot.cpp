/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"
#include "Debug.h"


#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>


Robot* s_Nova;

// constexpr double kRamseteB = 2;
// constexpr double kRamseteZeta = 0.7;


SwerveDrive* Robot::GetSwerveDrive() {
  return m_SwerveDrive;
}



void Robot::RobotInit() {
  s_Nova = this;
  initDebugging();


  m_SwerveDrive = new SwerveDrive({0, 4}, {1, 5}, {2, 6}, {3, 7});
  m_Joy = new frc::Joystick(0);
  m_JoyWheel = new frc::Joystick(1);

//   std::cout << "dc\n";
//   drive = new DriveController();
//   shooter = new Shooter();
//   arm = new Arm();
//   intake = new Intake();
//   controlpanel = new ControlPanel();

//   std::cout << "robo_init\n";
//   a_drive = new AutonDrive(drive, drive->ahrs);
//   std::cout << "auto init complete\n";
//   m_container = new RobotContainer(a_drive, shooter, arm, intake);
//   std::cout << "robo container init complete\n";
  
// /*   m_descolor_chooser.AddDefault("None",  Colors::WHITE);
//   m_descolor_chooser.AddObject("Red",    Colors::RED);
//   m_descolor_chooser.AddObject("Blue",   Colors::BLUE); 
//   m_descolor_chooser.AddObject("Green",  Colors::GREEN);
//   m_descolor_chooser.AddObject("Yellow", Colors::YELLOW);*/
//   m_container->InitAutoChoices();
//   std::cout << "auto choices init complete\n";

//   frc::SmartDashboard::PutData("Auto Modes", &(m_container->m_chooser));
// /*
//   m_descolor_chooser.AddDefault("None",  Colors::WHITE);
//   m_descolor_chooser.AddObject("Red",    Colors::RED);
//   m_descolor_chooser.AddObject("Blue",   Colors::BLUE);
//   m_descolor_chooser.AddObject("Green",  Colors::GREEN);
//   m_descolor_chooser.AddObject("Yellow", Colors::YELLOW);



// */
// /*d
//   Falcon_T = new TalonFX(0);
//   Falcon_T2 = new TalonFX(1);

//   T46 = new TalonSRX(46);
//   T46->SetInverted(InvertType::InvertMotorOutput);
// */
//   JoyThrottle = new frc::Joystick(0);
//   JoyWheel = new frc::Joystick(1);

//   m_descolor_chooser.AddDefault("None",  Colors::WHITE);
//   m_descolor_chooser.AddObject("Red",    Colors::RED);
//   m_descolor_chooser.AddObject("Blue",   Colors::BLUE);
//   m_descolor_chooser.AddObject("Green",  Colors::GREEN);
//   m_descolor_chooser.AddObject("Yellow", Colors::YELLOW);

//   frc::SmartDashboard::PutData("Desired Color", &m_descolor_chooser);
//   // talon0 = new TalonSRX(0);

//   cs::UsbCamera camera = frc::CameraServer::GetInstance()->StartAutomaticCapture("coolmethgames.gov", 0);

// 	camera.SetResolution(320, 190);
// 	// camera.SetExposureManual(0);
// 	// camera.SetBrightness(100);


//   tsm = new TeleopStateMachine(shooter, intake, controlpanel, arm);
  
//   JoyOp = new frc::Joystick(2);
}

void Robot::RobotPeriodic() {
  if (m_Joy->GetRawButton(11)) {
    m_SwerveDrive->ZeroEncoders();
    std::cout<< "Zeroed\n";
  }
  // frc2::CommandScheduler::GetInstance().Run();

}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() {
  // std::cout << "as get\n";
  // m_container->m_autoSelected = m_container->m_chooser.GetSelected();
  // // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  // //     kAutoNameDefault);
  // std::cout << "Auto selected: " << m_container->m_autoSelected << std::endl;

  // if (m_autonomousCommand != nullptr) {
  //   m_autonomousCommand->Cancel();
  //   m_autonomousCommand = nullptr;
  // }

  // m_autonomousCommand = m_container->GetAutonomousCommand();
  // std::cout << "ac gotten\n";

  // if (m_autonomousCommand != nullptr) {
  //   m_autonomousCommand->Schedule();
  //   std::cout << "ac schedule\n";
  // }


  // // a_drive->ResetOdometry();
}

void Robot::AutonomousPeriodic() {
  // // if (m_container->m_autoSelected == kAutoNameCustom) {
  // //   // Custom Auto goes here
  // // } else {
  // //   // Default Auto goes here
  // // }

  // // a_drive->Update();

  // frc::Pose2d pose = a_drive->GetPose();

  // frc::SmartDashboard::PutNumber("heading", pose.Rotation().Degrees().value());
  // frc::SmartDashboard::PutNumber("translation x", pose.Translation().X().value());
  // frc::SmartDashboard::PutNumber("translation y", pose.Translation().Y().value());



  // shooter->ShooterStateMachine();
  // arm->IntakeArmStateMachine();
  // intake->IntakeStateMachine();
  // // pose.


}
void Robot::TeleopInit() {
  // frc2::CommandScheduler::GetInstance().Disable();
  // if (m_autonomousCommand != nullptr) {
  //   m_autonomousCommand->Cancel();
  //   m_autonomousCommand = nullptr;
  // }

  // drive->ResetConfigs();

  frc::Shuffleboard::SelectTab("Drive");

  m_SwerveDrive->StopAll();
}

void Robot::TeleopPeriodic() {

  m_SwerveDrive->Update(m_Joy, m_JoyWheel);

  

  // // if (joyT->GetRawButton(BUTTON_STOP)) {
  // //   controlpanel->Stop();
  // // }
  // // if (joyT->GetRawButton(POSITION_BUTTON)) {
  // //   controlpanel->PositionMode();
  // // }
  // // if (joyT->GetRawButton(ROTATION_BUTTON)) {
  // //   controlpanel->RotationMode();
  // // }

  // // frc::SmartDashboard::PutNumber("speed", joyT->GetThrottle());

  // // if (joyT->GetRawButton(1)) {
  // //   Falcon_T->Set(ControlMode::PercentOutput, joyT->GetThrottle());
  // //   Falcon_T2->Set(ControlMode::PercentOutput, joyT->GetThrottle());
  // // } else {
  // //   Falcon_T->Set(ControlMode::PercentOutput, 0);
  // //   Falcon_T2->Set(ControlMode::PercentOutput, 0);
  // // }

  // drive->RunTeleopDrive(JoyThrottle, JoyWheel, true, false, false);
  // tsm->StateMachine(tsm->GatherButtonDataFromJoysticks(
  //   JoyThrottle, JoyWheel, JoyOp)); //joyOp maybe???
  // // drive->ManualOpenLoopDrive(joyT, joyW);
  // // drive->TeleopWCDrive(joyT,joyW,false,false);

  // // T46->Set(ControlMode::PercentOutput, 1.0f);

  // // if (joyT->GetRawButton(1)) {

  // // } else {
  // //   T46->Set(ControlMode::PercentOutput, 0.0f);
  // // }
  // // controlpanel->StateMachine();

  // // if (((currentColor == desiredColor || desiredColor == Colors::WHITE) && !joy->GetTrigger()) || joy->GetRawButton(2)) {
  // //   talon0->Set(ControlMode::PercentOutput, 0);
  // // } else {
  // //   talon0->Set(ControlMode::PercentOutput, CONTROL_WHEEL_SPEED_ON);

  // // frc::SmartDashboard::PutNumber("Speed", joy->GetThrottle());
  // frc::SmartDashboard::PutString("Last State", TeleopStateMachine::StateName(tsm->last_state));
  // frc::SmartDashboard::PutString("Current State", TeleopStateMachine::StateName(tsm->state));
}

// void Robot::UpdateButtons(){

//   // rest = joy->GetRawButton(9);
//   // down = joy->GetRawButton(8);
//   // up = joy->GetRawButton(7);
  
  
// }

void Robot::TestPeriodic() {}




#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
