/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Arm.h"

#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>
#include <math.h>

double kP = 0.8, kI = 0, kD = 0.0, kIz = 0, kFF = 0, kMaxOutput = 0.3, kMinOutput = -0.3;


Arm::Arm() {

//  talonArm = new TalonSRX(1); uh maybe talon for intake not arm?
  armSparkM0 = new rev::CANSparkMax(21, rev::CANSparkMax::MotorType::kBrushless); //RIGHT SIDE Leader
  armEncoder = new rev::CANEncoder(armSparkM0->GetEncoder());
  armPID = new rev::CANPIDController(armSparkM0->GetPIDController());
  analog = new rev::CANAnalog(armSparkM0->GetAnalog());
  armSparkM1 = new rev::CANSparkMax(24, rev::CANSparkMax::MotorType::kBrushless);//LEFT SIDE Follower
  // rev::CANSparkMax armSpark0{1, rev::CANSparkMax::MotorType::kBrushless};
  armSparkM0->RestoreFactoryDefaults();
  armSparkM1->RestoreFactoryDefaults();
  armSparkM0->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  armSparkM1->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  
  armSparkM1->Follow(*armSparkM0, true); //, true

  armStartPos = armEncoder->GetPosition();
  targetPos = armStartPos;

  armPID->SetP(kP);
  armPID->SetI(kI);
  armPID->SetD(kD);
  armPID->SetIZone(kIz);
  armPID->SetFF(kFF);
  armPID->SetOutputRange(kMinOutput, kMaxOutput);

  armSparkM0->BurnFlash();
  armSparkM1->BurnFlash();

  // display PID coefficients on SmartDashboard
  frc::SmartDashboard::PutNumber("P Gain", kP);
  frc::SmartDashboard::PutNumber("I Gain", kI);
  frc::SmartDashboard::PutNumber("D Gain", kD);
  frc::SmartDashboard::PutNumber("I Zone", kIz);
  frc::SmartDashboard::PutNumber("Feed Forward", kFF);
  frc::SmartDashboard::PutNumber("Max Output", kMaxOutput);
  frc::SmartDashboard::PutNumber("Min Output", kMinOutput);
  frc::SmartDashboard::PutNumber("s", analog->GetVoltage());
  // frc::SmartDashboard::PutNumber(armSparkM1->getv, 0);
  
  //armPID->SetSmartMotionAccelStrategy(rev::CANPIDController::AccelStrategy::kSCurve);


}

void Arm::Up() {

  // talonArm->Set(ControlMode::PercentOutput, 0.3);
  // armSparkM0->Set(0.1);
  // if(armCurrPos >= armStartPos){

  //   // if(armCurrPos <= armStartPos + 0.2){
  //     armSparkM0->Set(-0.025);
  //   // } else {
  //   //   armSparkM0->Set(0);
  //   // }

  // } else {
  //   armSparkM0->Set(0);
  // }
   //armPID->SetReference(armStartPos, rev::ControlType::kPosition);
  MoveToPosition(armStartPos+START_OFFSET);
  //  armSparkM0->Set(1.0f);
}

void Arm::Down() {

  // talonArm->Set(ControlMode::PercentOutput, -0.3);
  // armSparkM0->Set(-0.1);
  // if(armStartPos + 0.25 >= armCurrPos){

  //   if(armCurrPos >= armStartPos - .0625){
  //     armSparkM0->Set(0.025);
  //   } else {
  //    armSparkM0->Set(0);
  //   }

  // } else {
  //   armSparkM0 ->Set(0);
  // }
  //armPID->SetReference(armStartPos+5, rev::ControlType::kPosition);
  MoveToPosition(armStartPos+DOWN_POS);
  //armSparkM0->Set(-1.0f);
}

void Arm::Rest() {

  // talonArm->Set(ControlMode::PercentOutput, 0.0);
  armSparkM0->Set(0);
}

void Arm::MoveToPosition(double desiredPosition){
  frc::SmartDashboard::PutNumber("intake desired position", desiredPosition);
  frc::SmartDashboard::PutNumber("intake position error", desiredPosition - (armEncoder->GetPosition()));
  armPID->SetReference(desiredPosition, rev::ControlType::kPosition);
}

// void Arm::ShouldBeInSmartMode() {
//   if (abs(m_drive->GetAngularSpeed()) >= SMART_MODE_THRESHOLD_ANG_SPEED) {
//     return true;
//   }
//   if (abs(m_drive->GetAngularAcceleration()) >= SMART_MODE_THRESHOLD_ANG_SPEED) {
//     return true;
//   }
//   if (abs(m_drive->GetForwardSpeed()) >= SMART_MODE_THRESHOLD_ANG_SPEED) {
//     return true;
//   }
//   if (abs(m_drive->GetForwardAcceleration()) >= SMART_MODE_THRESHOLD_ANG_SPEED) {
//     return true;
//   }


// }

void Arm::IntakeArmStateMachine() {
  armCurrPos = armEncoder->GetPosition();
  frc::SmartDashboard::PutNumber("INTAKE ARM STATE", intake_arm_state);
  frc::SmartDashboard::PutNumber("Rotation", armEncoder->GetPosition()); //in rot'n
  frc::SmartDashboard::PutNumber("start", armStartPos);
  frc::SmartDashboard::PutNumber("arm output", armSparkM0->GetAppliedOutput());
  // frc::SmartDashboard::PutNumber("Degrees", modf(armEncoder->GetPosition(), nullptr) * 360);// in degrees
/* 
  double p = frc::SmartDashboard::GetNumber("P Gain", 0);
  double i = frc::SmartDashboard::GetNumber("I Gain", 0);
  double d = frc::SmartDashboard::GetNumber("D Gain", 0);
  double iz = frc::SmartDashboard::GetNumber("I Zone", 0);
  double ff = frc::SmartDashboard::GetNumber("Feed Forward", 0);
  double max = frc::SmartDashboard::GetNumber("Max Output", 0);
  double min = frc::SmartDashboard::GetNumber("Min Output", 0);
  double rotations = frc::SmartDashboard::GetNumber("Set Rotations", 0);

  // if PID coefficients on SmartDashboard have changed, write new values to controller
  if((p != kP)) { armPID->SetP(p); kP = p; }
  if((i != kI)) { armPID->SetI(i); kI = i; }
  if((d != kD)) { armPID->SetD(d); kD = d; }
  if((iz != kIz)) { armPID->SetIZone(iz); kIz = iz; }
  if((ff != kFF)) { armPID->SetFF(ff); kFF = ff; }
  if((max != kMaxOutput) || (min != kMinOutput)) {
    armPID->SetOutputRange(min, max);
    kMinOutput = min; kMaxOutput = max;
  } */

  switch(intake_arm_state){

    case REST_STATE:
      if (last_intake_arm_state != REST_STATE) {
        Rest();
      }
      frc::SmartDashboard::PutString("INTAKE ARM", "rest");
    break;

    case UP_STATE:
      if (last_intake_arm_state != UP_STATE) {
        targetPos = armStartPos;
        MoveToPosition(armStartPos + START_OFFSET);
      }
      frc::SmartDashboard::PutString("INTAKE ARM", "up");
    break;

    case DOWN_STATE:
      if (last_intake_arm_state != DOWN_STATE) {
        targetPos = armStartPos + DOWN_POS;
      }
      if (abs(targetPos - armCurrPos) > 0){
         MoveToPosition(targetPos);
      } else {
        Rest();
      }
      frc::SmartDashboard::PutString("INTAKE ARM", "down");
    break;

  }

  last_intake_arm_state = intake_arm_state;

}
  