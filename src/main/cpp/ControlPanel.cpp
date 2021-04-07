#include "ControlPanel.h"
#include <frc/smartdashboard/SmartDashboard.h>

const double CPAkP = 0.8, CPAkI = 0, CPAkD = 0.0, CPAkIz = 0, CPAkFF = 0, CPAkMaxOutput = 0.3, CPAkMinOutput = -0.3;
const double CPRkP = 0.8, CPRkI = 0, CPRkD = 0.0, CPRkIz = 0, CPRkFF = 0, CPRkMaxOutput = 0.3, CPRkMinOutput = -0.3;

ControlPanel::ControlPanel() {
  // talon = new TalonSRX(CONTROL_PANEL_TALON);
  // controlPanelArm = new rev::CANSparkMax(controlPanelArmSpark, rev::CANSparkMax::MotorType::kBrushless);
  // controlPanelArmEncoder = new rev::CANEncoder(controlPanelArm->GetEncoder());
  // controlPanelArmPID = new rev::CANPIDController(controlPanelArm->GetPIDController());
  // controlPanelRotator = new rev::CANSparkMax(controlPanelRotatorSpark, rev::CANSparkMax::MotorType::kBrushless);
  // controlPanelRotatorEncoder = new rev::CANEncoder(controlPanelRotator->GetEncoder());
  // controlPanelRotatorPID = new rev::CANPIDController(controlPanelRotator->GetPIDController());
  
  // controlPanelArmStartPos = controlPanelArmEncoder->GetPosition();
  // controlPanelWheelStartPos = controlPanelRotatorEncoder->GetPosition();
  // controlPanelArmPID->SetP(CPAkP);
  // controlPanelArmPID->SetI(CPAkI);
  // controlPanelArmPID->SetD(CPAkD);
  // controlPanelArmPID->SetIZone(CPAkIz);
  // controlPanelArmPID->SetFF(CPAkFF);
  // controlPanelArmPID->SetOutputRange(CPAkMinOutput, CPAkMaxOutput);

  // controlPanelRotatorPID->SetP(CPRkP);
  // controlPanelRotatorPID->SetI(CPRkI);
  // controlPanelRotatorPID->SetD(CPRkD);
  // controlPanelRotatorPID->SetIZone(CPRkIz);
  // controlPanelRotatorPID->SetFF(CPRkFF);
  // controlPanelRotatorPID->SetOutputRange(CPRkMinOutput, CPRkMaxOutput);

}

// std::string ControlPanel::getColor(Colors c) {
//   switch (c) {
//     case Colors::RED:
//       return "Red";
//     case Colors::YELLOW:
//       return "Yellow";
//     case Colors::BLUE:
//       return "Blue";
//     case Colors::GREEN:
//       return "Green";
//     case Colors::WHITE:
//       return "None";
//     default:
//       return "None";
//   }
// }

// Colors ColorFromFRCColor(frc::Color detectedColor) {
//     frc::SmartDashboard::PutNumber("Red", detectedColor.red);
//     frc::SmartDashboard::PutNumber("Green", detectedColor.green);
//     frc::SmartDashboard::PutNumber("Blue", detectedColor.blue);

//     if (detectedColor.red > 0.3 && detectedColor.green > 0.5 && detectedColor.blue < 0.2) {
//       frc::SmartDashboard::PutString("Color", "Yellow");
//       return Colors::YELLOW;
//     } else if (detectedColor.red > 0.4) {
//       frc::SmartDashboard::PutString("Color", "Red");
//       return Colors::RED;
//     } else if (detectedColor.green > 0.5 && detectedColor.blue < 0.3 && detectedColor.red < 0.2 && detectedColor.blue < 0.3) {
//       frc::SmartDashboard::PutString("Color", "Green");
//       return Colors::GREEN;
//     } else if (detectedColor.blue > 0.4 && detectedColor.red < 0.2 && detectedColor.green > 0.4) {
//       frc::SmartDashboard::PutString("Color", "Blue");
//       return Colors::BLUE;
//     } else {
//       frc::SmartDashboard::PutString("Color", "None");
//       return Colors::WHITE;
//     }
// }

// void ControlPanel::Rotate() {
//   // talon->Set(ControlMode::PercentOutput, CONTROL_PANEL_SPEEN_ON);
//   controlPanelRotatorPID->SetReference(controlPanelGoSpeed, rev::ControlType::kVelocity);  
// }

// void ControlPanel::Stop() {
//   // talon->Set(ControlMode::PercentOutput, 0);
//   controlPanelRotatorPID->SetReference(controlPanelStopSpeed, rev::ControlType::kVelocity);
//   state = IDLE;
// }

void ControlPanel::StateMachine() {
    
    // detectedColor = m_colorSensor.GetColor();

    switch(state) {
        case IDLE:
          // controlPanelArm->Set(0);
          // controlPanelRotator->Set(0);
          last_state = States::IDLE;
          break;
        case POSITION_MODE:
          // controlPanelArmPID->SetReference(controlPanelArmStartPos+armRaisePos, rev::ControlType::kPosition);
          // if (!HasReachedPosition(ColorFromFRCColor(detectedColor))) {
          //     Rotate();
          // } else {
          //     Stop();
          // }
          last_state = States::POSITION_MODE;
          break;
        case ROTATION_MODE:
          // if(last_state != ROTATION_MODE){
          //   controlPanelWheelStartPos = controlPanelRotatorEncoder->GetPosition();
          // }
          // controlPanelArmPID->SetReference(controlPanelArmStartPos+armRaisePos, rev::ControlType::kPosition);
          // controlPanelRotatorPID->SetReference(controlPanelWheelStartPos+rotationModeRot, rev::ControlType::kPosition);
          
          last_state = States::ROTATION_MODE;
          break;
        case HUMAN_LOAD:
          // controlPanelArmPID->SetReference(controlPanelArmStartPos+armRaisePos, rev::ControlType::kPosition);
          last_state = States::HUMAN_LOAD;
    }
}
