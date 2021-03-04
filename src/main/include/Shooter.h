#pragma once

#include <string>

#include <frc/smartdashboard/SmartDashboard.h>
#include <ctre/Phoenix.h>
#include <frc/Joystick.h>
#include <rev/CANSparkMax.h>
#include "Macros.h"
#include "Constants.h"

#define SHOOTER_SPEED 594.3893301
#define MOTOR_ROTATION_PER_HALF_BELT_ROTATION 26.1799387799

class Shooter {
    public:

    // const int topRollerSpark = 2;
    // const int bottomRollerSpark = 3;
    double shootkP = 10.8, shootkI = 0, shootkD = 0.0, shootkIz = 0, shootkFF = 0, shootkMaxOutput = 0.3, shootkMinOutput = -0.3; // Not tuned yet
    double beltkP = 10.8, beltkI = 0, beltkD = 0.0, beltkIz = 0, beltkFF = 0, beltkMaxOutput = 0.3, beltkMinOutput = -0.3;
    double indexerkP = 10.8, indexerkI = 0, indexerkD = 0.0, indexerkIz = 0, indexerkFF = 0, indexerkMaxOutput = 0.3, indexerkMinOutput = -0.3;

    
    const double msPerFalconUpdate = 100.0;
    const double msPerSecond = 1000.0;

    const double falconUpdatePerSecond = msPerFalconUpdate / msPerSecond;

    const int ticksPerRot = 2048;
    const double ticksToRad = (2 * apes::PI) / ticksPerRot;
    const double radsToTicks = (1024 / apes::PI);

    const int beltSpeed = 668;
    const double topShootSpeed = SHOOTER_SPEED * radsToTicks * 2;
    const double bottomShootSpeed = SHOOTER_SPEED * radsToTicks * 1;
    const double indexerShootSpeed = 668.0f;

    const double farModifier = 1;
    const double mediumModifier = 1;
    const double closeModifier = 1;

    enum ShooterState {
        INIT_STATE, INTAKE_STATE, STOP_STATE, 
        FAR_SHOOT_STATE, MEDIUM_SHOOT_STATE, CLOSE_SHOOT_STATE, 
        WAITING_STATE, REVERSE_STATE
    };
   
    ShooterState last_shooter_state = INIT_STATE;
    ShooterState shooter_state = INIT_STATE;

    double belt_position;

    const int beltSparkID = 668;
    const int rightindexerSparkID = 668;
    const int leftindexerSparkID = 668;

    const int topWTalonID = 668;
    const int bottomWTalonID = 668;

    rev::CANSparkMax *beltSpark, *leftIndexerSpark, *rightIndexerSpark;
    rev::CANPIDController *beltPID, *leftIndexerPID, *rightIndexerPID;
    rev::CANEncoder *beltEncoder, *leftIndexerEncoder, *rightIndexerEncoder;

    frc::Joystick* joy;

    WPI_TalonFX* topWTalon;
    WPI_TalonFX* bottomWTalon;

    Shooter();
    void Init();
    void FarShoot();
    void MediumShoot();
    void CloseShoot();
    void Intake();
    void Stop();
    void Waiting();
    void Reverse();
    void ConfigureSpark(rev::CANSparkMax *spark, rev::CANPIDController *PIDController, double kP, double kI, double kD, 
        double kIz, double kFF, double minOut, double maxOut, rev::CANSparkMax::IdleMode idleMode);
    void ConfigureTalon(WPI_TalonFX *talon, int pidSlot, int pidType, int timeoutMs, bool inverted,  
        int kF, int kP, int kI, int kD, NeutralMode brakeMode);

    void ShooterStateMachine();
};

