#pragma once
// #ifndef SRC_INTAKE_H_
// #define SRC_INTAKE_H_
#pragma once


#include <ctre/Phoenix.h>
#include <frc/WPILib.h>
// #include "Robot.h"
#include "rev/CANSparkMax.h"
#include "Arm.h"
#include "Shooter.h"

class Intake {
public:

  // TalonSRX *talonIntake;
  rev::CANSparkMax *intakeSparkM0;


  enum States {
    STOP_STATE, IN_STATE, OUT_STATE 
  };

  States intake_state = STOP_STATE;

  const int intakeWheelSpark = 22;

  const float IDLE_WHEEL_SPEED = 0.1;
  const float IN_WHEEL_SPEED = 0.35;
  const float OUT_WHEEL_SPEED = -0.35;

    Intake();

    void Stop();
    void In();
    void Out();

    void IntakeStateMachine();



};

// #endif
