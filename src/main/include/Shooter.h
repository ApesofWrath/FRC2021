#pragma once

#include <string>

//#include <frc/WPILib.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <ctre/Phoenix.h>
#include <frc/Joystick.h>
#include "rev/CANSparkMax.h"
#include "Macros.h"

#define SHOOTER_SPEED 594.3893301

class Shooter {
    public:

    const int beltSpark = 1;
    const int topRollerSpark = 2;
    const int bottomRollerSpark = 3;

    const int topShootSpeed = ((SHOOTER_SPEED)*60)/(2*PI)/2;
    const int bottomShootSpeed = ((SHOOTER_SPEED)*60)/(2*PI);

    const int INIT_STATE_H = 0;
    const int INTAKE_STATE_H = 1;
    const int STOP_STATE_H = 2;
    const int SHOOT_STATE_H = 3;
    const int WAITING_STATE_H = 4;
    const int REVERSE_STATE_H = 5;
    
    const int ONEROT_STATE_H = 5;

    int last_shooter_state = INIT_STATE_H;
    int shooter_state = INIT_STATE_H;

    float speed = 0;
    float beltPOS;

    // TalonSRX *canTalonBelt, *canTalonTopW, *canTalonBottomW;
    rev::CANSparkMax *beltNEO, *topWNEO, *botWNEO;
    rev::CANPIDController *beltPID, *topWPID, *botWPID;
    rev::CANEncoder *beltEncoder, *topWEncoder, *botWEncoder;
    frc::Joystick* joy;

    Shooter();
    void Shoot();
    void Intake();
    void Stop();
    void Waiting();
    void Reverse();

    void ShooterStateMachine();
};

