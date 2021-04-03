#pragma once
// #ifndef SRC_INTAKE_H_
// #define SRC_INTAKE_H_
#pragma once


#include <ctre/Phoenix.h>
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

  const double IDLE_WHEEL_SPEED = 0.1;
  const double IN_WHEEL_SPEED = 0.35;
  const double OUT_WHEEL_SPEED = -0.35;

    Intake();

    void Stop();
    void In();
    void Out();

    void IntakeStateMachine();



};

// #endif
