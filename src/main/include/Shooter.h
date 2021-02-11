#pragma once

#include <string>

#include <frc/smartdashboard/SmartDashboard.h>
#include <ctre/Phoenix.h>
#include <frc/Joystick.h>
#include "rev/CANSparkMax.h"
#include "Macros.h"
#include "Constants.h"

#define SHOOTER_SPEED 594.3893301

class Shooter {
    public:

    // const int topRollerSpark = 2;
    // const int bottomRollerSpark = 3;

    const int topShootSpeed = ((SHOOTER_SPEED) * 60) / (2 * apes::PI) / 2;
    const int bottomShootSpeed = ((SHOOTER_SPEED) * 60) / (2 * apes::PI);

    const int frontBeltSpeed = 668;
    const int backBeltSpeed = 668;

    enum ShooterState {
        INIT_STATE, INTAKE_STATE, STOP_STATE, 
        SHOOT_STATE, WAITING_STATE
    };
    
    // const int ONEROT_STATE_H = 5;

    ShooterState last_shooter_state = INIT_STATE;
    ShooterState shooter_state = INIT_STATE;

    double speed = 0;
    double belt_position;


    const int frontBeltSpark = 668;
    const int backBeltSpark = 668;
    const int indexerTalonID = 668;
    const int topWTalonID = 668;
    const int bottomWTalonID = 668;

    rev::CANSparkMax *frontBeltNEO, *backBeltNEO;
    rev::CANPIDController *frontBeltPID, *backBeltPID;
    rev::CANEncoder *frontBeltEncoder, *backBeltEncoder;

    frc::Joystick* joy;
    
    TalonSRX* indexerTalon;
    WPI_TalonFX* topWTalon;
    WPI_TalonFX* bottomWTalon;
    // const int INIT_STATE_H = 0;
    // const int INTAKE_STATE_H = 1;
    // const int STOP_STATE_H = 2;
    // const int SHOOT_STATE_H = 3;
    // const int WAITING_STATE_H = 4;

    Shooter();
    void Init();
    void Shoot();
    void Intake();
    void Stop();
    void Waiting();
    void Reverse();

    void ShooterStateMachine();
};

