#pragma once

#include <string>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <ctre/Phoenix.h>
#include <frc/Joystick.h>
#include <rev/CANSparkMax.h>
#include "Macros.h"
#include "Constants.h"

#define SHOOTER_SPEED 594.3893301
#define MOTOR_ROTATION_PER_HALF_BELT_ROTATION 26.1799387799

enum Distances{
        CLOSE = 0,
        MEDIUM = 1,
        FAR = 2
};

class Shooter {
    public:

    // const int topRollerSpark = 2;
    // const int bottomRollerSpark = 3;
    double k_P_shooter = 0.00015, k_I_shooter = 0, k_D_shooter = 0.0, k_Iz_shooter = 0, k_FF_shooter = 0.000022, k_max_out_shooter = 0.95f, k_min_out_shooter = -0.95f; // Not tuned yet
    double k_P_belt = 10.8, k_I_belt = 0, k_D_belt = 0.0, k_Iz_belt = 0, k_FF_belt = 0, k_max_out_belt = 0, k_min_out_belt = 0;
    double k_P_indexer = 10.8, k_I_indexer = 0, k_D_indexer = 0.0, k_Iz_indexer = 0, k_FF_indexer = 0, k_max_out_indexer = 0, k_min_out_indexer = 0;

    
    const double ms_per_falcon_updates = 100.0;
    const double ms_per_second = 1000.0;

    const double faclcon_updates_per_second = ms_per_falcon_updates / ms_per_second;

    const int ticks_per_rotation = 2048;
    const double ticks_to_radians = (2 * apes::PI) / ticks_per_rotation;
    const double radians_to_ticks = (1024 / apes::PI);

    const int belt_speed = 668;
    const double top_shoot_speed = SHOOTER_SPEED * radians_to_ticks * 2;
    const double bottom_shoot_speed = SHOOTER_SPEED * radians_to_ticks * 1;
    const double indexer_shoot_speed = 668.0f;

    const double far_modifier = 1;
    const double medium_modifier = 1;
    const double close_modifier = 1;

    enum ShooterState {
        INIT_STATE, INTAKE_STATE, STOP_STATE, 
        FAR_SHOOT_STATE, MEDIUM_SHOOT_STATE, CLOSE_SHOOT_STATE, 
        WAITING_STATE, REVERSE_STATE
    };

    frc::SendableChooser<Distances> distance_chooser;

    ShooterState last_shooter_state = INIT_STATE;
    ShooterState shooter_state = INIT_STATE;

    double belt_position;

    const int belt_spark_ID = 668;
    const int right_indexer_spark_ID = 668;
    const int left_indexer_spark_ID = 668;

    const int top_wheel_spark_ID = 668;
    const int bottom_wheel_spark_ID = 668;

    rev::CANSparkMax *belt_spark, *left_indexer_spark, *right_indexer_spark, *top_wheel_spark, *bottom_wheel_spark;
    rev::CANPIDController *belt_PID, *left_indexer_PID, *right_indexer_PID, *top_wheel_PID, *bottom_wheel_PID;
    rev::CANEncoder *belt_encoder, *left_indexer_encoder, *right_indexer_encoder, *top_wheel_encoder, *bottom_wheel_encoder;

    frc::Joystick* joy;
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

