#include "RobotContainer.h"
#include <units/units.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/smartdashboard/SmartDashboard.h>


#include <iostream>
#include <vector>

RobotContainer::RobotContainer(Shooter* shooter, Arm* arm, Intake* intake, SwerveDrive *swerve) {
    m_shooter = shooter;
    m_arm = arm;
    m_intake = intake;
    m_shooter_subsystem = new ShooterSubsystem(m_shooter, m_joystick);
    m_swerve_subsystem = new SwerveSubsystem(m_swerve, m_joystick);

    frc2::CommandScheduler::GetInstance().OnCommandInitialize(
      [](const frc2::Command& command) {
        frc::SmartDashboard::PutString(
            "Command Initialized", command.GetName());
      });
        

    // frc2::CommandScheduler::GetInstance().

    // frc2::CommandScheduler::GetInstance().
    ConfigureButtonBindings();
}

const frc::DifferentialDriveKinematics K_DRIVE_KINEMATICS{
        units::meter_t(K_TRACK_WIDTH)
    };


frc2::Command* RobotContainer::GetAutonomousCommand() {

    // Setup
    // frc::SwerveDriveKinematicsConstraint *swerve_constraints = new 

    // frc::DifferentialDriveVoltageConstraint *autoVoltageConstraint = new frc::DifferentialDriveVoltageConstraint(
    //   frc::SimpleMotorFeedforward<units::meter>(
    //       K_S, K_V, K_A),
    //   K_DRIVE_KINEMATICS, units::volt_t(10));

    frc::TrajectoryConfig *config = new frc::TrajectoryConfig(units::meters_per_second_t(2),
                                units::meters_per_second_squared_t(3));
    config->SetKinematics(K_DRIVE_KINEMATICS);
    // config->AddConstraint(*autoVoltageConstraint);

    // Pathweaver load    NOT YET
    // wpi::SmallString<64> deployDirectory;
    // frc::filesystem::GetDeployDirectory(deployDirectory);
    // wpi::sys::path::append(deployDirectory, "paths");
    // wpi::sys::path::append(deployDirectory, m_autoSelected + ".wpilib.json");
      // frc::Trajectory trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory);
 

    std::cout << "selecting time\n";
    // Hardcode
    frc::Pose2d start, end;
    std::vector<frc::Translation2d> points;
    m_autoSelected = CROSS_INIT_LINE;
    frc::Trajectory trajectory, trajectory1, trajectory2, trajectoryTurn;
    const frc::TrapezoidProfile<units::radians>::Constraints
        kThetaControllerConstraints{};
    // /\ not real put in right numbers (max velo, max accel)

    // frc2::SwerveControllerCommand<4> *swerve_command; //, *ramseteCommand2, *ramseteCommandTurn;

    if(CROSS_INIT_LINE == m_autoSelected){
            std::cout << "cil\n";
            start = frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg));
            end = frc::Pose2d(0.5_m, 0_m, frc::Rotation2d(0_deg));
            points = {};
            std::cout << "cil2\n";
            trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
                start,
                points,
                end,
                *config);
            frc2::SwerveControllerCommand<4> swerve_command(
                    trajectory, 
                    [this]() { return m_swerve_subsystem->GetPose(); },
                    m_swerve_subsystem->m_kinematics,
                    frc2::PIDController(K_P_LEFT_VEL, 0, 0), // x contrller
                    frc2::PIDController(K_P_RIGHT_VEL, 0, 0), // y controller
                    frc::ProfiledPIDController<units::radians>(0, 0, 0, kThetaControllerConstraints, 20_ms), // yaw controller
                    [this](auto swerve_states){ m_swerve_subsystem->SetModuleStates(swerve_states); },
                    {m_swerve_subsystem}
                );
            m_swerve_subsystem->BuildOdometry(start);
            return new frc2::SequentialCommandGroup(
                std::move(swerve_command)
                
                // frc2::InstantCommand([this] { m_drive->TankDriveVolts(0_V, 0_V); }, {})
            );


        // case SHOOT_PRELOAD:
        //     std::cout << "sp\n";
        //     start = frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg));
        //     end = frc::Pose2d(-3.048_m, 0_m, frc::Rotation2d(0_deg));
        //     points = {
        //     };
        //     config->SetReversed(true);
        //     std::cout << "sp2\n";
        //     trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
        //         start,
        //         points,
        //         end,
        //         *config);
        //     ramseteCommand = new frc2::RamseteCommand(
        //         trajectory, [this]() { return m_drive->GetPose(); },
        //         frc::RamseteController(K_RAMSETE_B,
        //                                 K_RAMSETE_ZETA),
        //         frc::SimpleMotorFeedforward<units::meters>(
        //             K_S, K_V, K_A),
        //         K_DRIVE_KINEMATICS,
        //         [this] { return m_drive->GetWheelSpeeds(); },
        //         frc2::PIDController(K_P_LEFT_VEL, 0, 0),
        //         frc2::PIDController(K_P_RIGHT_VEL, 0, 0),
        //         [this](auto left, auto right) { m_drive->TankDriveVolts(left, right); },
        //         {m_drive});
        //     m_drive->ResetOdometry(start);  
        //     return new frc2::SequentialCommandGroup(
        //         std::move(*ramseteCommand),
        //         frc2::InstantCommand([this] { m_drive->TankDriveVolts(0_V, 0_V); }, {}),
        //         frc2::InstantCommand([this] { m_shooter->shooter_state = m_shooter->FAR_SHOOT_STATE;}),
        //         frc2::WaitCommand(1.5_s),
        //         frc2::InstantCommand([this] { m_shooter->shooter_state = m_shooter->STOP_STATE;})
        //     );
        // case FIVE_BALL:
        //     start = frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg));
        //     trajectory1 = frc::TrajectoryGenerator::GenerateTrajectory(
        //         start,
        //         {},
        //         frc::Pose2d(2.67_m, 0_m, frc::Rotation2d(0_deg)),
        //         *config
        //     );
        //     trajectoryTurn = frc::TrajectoryGenerator::GenerateTrajectory(
        //         frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
        //         {},
        //         frc::Pose2d(0_m, -0.01_m, frc::Rotation2d(-30_deg)),
        //         *config
        //     );
        //     config->SetReversed(true);
        //     trajectory2 = frc::TrajectoryGenerator::GenerateTrajectory(
        //         frc::Pose2d(2.67_m, 0_m, frc::Rotation2d(0_deg)),
        //         {},
        //         frc::Pose2d(-3.048_m, 5.08_m, frc::Rotation2d(0_deg)),
        //         *config
        //     );

        //     m_drive->ResetOdometry(start);

        //     ramseteCommand = new frc2::RamseteCommand(
        //         trajectory1, [this]() { return m_drive->GetPose(); },
        //         frc::RamseteController(K_RAMSETE_B,
        //                                 K_RAMSETE_ZETA),
        //         frc::SimpleMotorFeedforward<units::meters>(
        //             K_S, K_V, K_A),
        //         K_DRIVE_KINEMATICS,
        //         [this] { return m_drive->GetWheelSpeeds(); },
        //         frc2::PIDController(K_P_LEFT_VEL, 0, 0),
        //         frc2::PIDController(K_P_RIGHT_VEL, 0, 0),
        //         [this](auto left, auto right) { m_drive->TankDriveVolts(left, right); },
        //         {m_drive});
        //     ramseteCommandTurn = new frc2::RamseteCommand(
        //         trajectoryTurn, [this]() { return m_drive->GetPose(); },
        //         frc::RamseteController(K_RAMSETE_B,
        //                                 K_RAMSETE_ZETA),
        //         frc::SimpleMotorFeedforward<units::meters>(
        //             K_S, K_V, K_A),
        //         K_DRIVE_KINEMATICS,
        //         [this] { return m_drive->GetWheelSpeeds(); },
        //         frc2::PIDController(K_P_LEFT_VEL, 0, 0),
        //         frc2::PIDController(K_P_RIGHT_VEL, 0, 0),
        //         [this](auto left, auto right) { m_drive->TankDriveVolts(left, right); },
        //         {m_drive});
        //     ramseteCommand2 = new frc2::RamseteCommand(
        //         trajectory2, [this]() { return m_drive->GetPose(); },
        //         frc::RamseteController(K_RAMSETE_B,
        //                                 K_RAMSETE_ZETA),
        //         frc::SimpleMotorFeedforward<units::meters>(
        //             K_S, K_V, K_A),
        //         K_DRIVE_KINEMATICS,
        //         [this] { return m_drive->GetWheelSpeeds(); },
        //         frc2::PIDController(K_P_LEFT_VEL, 0, 0),
        //         frc2::PIDController(K_P_RIGHT_VEL, 0, 0),
        //         [this](auto left, auto right) { m_drive->TankDriveVolts(left, right); },
        //         {m_drive});
        //     return new frc2::SequentialCommandGroup(
        //         frc2::InstantCommand([this] {
        //             m_arm->intake_arm_state = m_arm->DOWN_STATE;
        //             m_intake->intake_state = m_intake->IN_STATE;
        //         }),
        //         std::move(*ramseteCommand),
        //         frc2::InstantCommand([this] { m_drive->TankDriveVolts(0_V, 0_V); }, {}),
        //         std::move(*ramseteCommandTurn),
        //         frc2::InstantCommand([this] { m_drive->TankDriveVolts(0_V, 0_V); }, {}),
        //         frc2::WaitCommand(0.5_s),
        //         frc2::InstantCommand([this] {
        //             m_arm->intake_arm_state = m_arm->UP_STATE;
        //             m_intake->intake_state = m_intake->STOP_STATE;
        //         }),
        //         std::move(*ramseteCommand2),
        //         frc2::InstantCommand([this] { m_drive->TankDriveVolts(0_V, 0_V); }, {}),
        //         frc2::InstantCommand([this] { m_shooter->shooter_state = m_shooter->FAR_SHOOT_STATE;}),
        //         frc2::WaitCommand(1.5_s),
        //         frc2::InstantCommand([this] { m_shooter->shooter_state = m_shooter->STOP_STATE;})
        //     );  
        // break;
    }else{
            std::cout << "df\n";
            return new frc2::InstantCommand([this] { frc::SmartDashboard::PutString("status","Why aren't you running auton??");});
    }
    }

void RobotContainer::ConfigureButtonBindings() {
    m_joystick = new frc::Joystick(2);
    frc2::JoystickButton(m_joystick, 1)
        .WhenPressed(new ShootHighCommand(m_shooter_subsystem, m_joystick));

    frc2::JoystickButton(m_joystick, 2)
        .WhenPressed(new ShootLowCommand(m_shooter_subsystem, m_joystick));
    // frc2::JoystickButton(&m_joystick, 1).WhenPressed(new ShootHigh())
}
/*

return new frc2::SequentialCommandGroup(
        frc2::InstantCommand([this] {
            m_arm->intake_arm_state = m_arm->DOWN_STATE;
            m_intake->intake_state = m_intake->IN_STATE;
            }),
        std::move(ramseteCommand),
        frc2::InstantCommand([this] { m_drive->TankDriveVolts(0_V, 0_V); }, {}),
        frc2::InstantCommand([this] {
            m_arm->intake_arm_state = m_arm->UP_STATE;
            m_intake->intake_state = m_intake->STOP_STATE;
            m_shooter->shooter_state = m_shooter->SHOOT_STATE_H;
            }),
        frc2::WaitCommand(1.5_s),
        frc2::InstantCommand([this] {
            m_shooter->shooter_state = m_shooter->STOP_STATE_H;
            })
        
        // frc2::InstantCommand([this] { m_shooter->DoOneBeltRotation(); }, {}),
        // frc2::WaitUntilCommand([this] () -> bool { return m_shooter->shooter_state == m_shooter->STOP_STATE_H;})
        
    );
*/
/*
change a state
frc2::InstantCommand([this] { m_intake->state = Intake::States::IN})

shoot




*/


void RobotContainer::InitAutoChoices() {
    m_chooser.SetDefaultOption("Cross Initiation Line", Auto::CROSS_INIT_LINE);
    m_chooser.AddOption("Outtake Preloaded", Auto::SHOOT_PRELOAD);
    m_chooser.AddOption("Five Bal", Auto::FIVE_BALL);
}
