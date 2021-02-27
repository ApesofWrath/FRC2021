// #pragma once

// #include <frc2/command/Command.h>
// #include <frc2/command/CommandBase.h>
// #include <frc2/command/SequentialCommandGroup.h>
// #include <frc2/command/InstantCommand.h>
// #include <frc2/command/WaitUntilCommand.h>
// #include <frc2/command/WaitCommand.h>
// #include <frc2/command/RamseteCommand.h>
// #include <frc/trajectory/TrajectoryConfig.h>
// #include <frc/trajectory/Trajectory.h>
// #include <frc/trajectory/TrajectoryUtil.h>
// #include <frc/trajectory/TrajectoryGenerator.h>

// #include "../Drive/DriveBase.h"
// #include "AutonDrive.h"
// #include "Shooter.h"
// #include "Arm.h"
// #include "Intake.h"
// #include "RobotContainer.h"

// #include <string>
// #include <vector>


// /*

//   frc2::RamseteCommand ramseteCommand(
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


//     std::vector<frc::Translation2d> points = {
//     };
//  frc2::InstantCommand([this] {
//         //     m_arm->intake_arm_state = m_arm->DOWN_STATE;
//         //     m_intake->intake_state = m_intake->IN_STATE;
//         //     }),


//         return new frc2::SequentialCommandGroup(
        
//         std::move(ramseteCommand),
//         frc2::InstantCommand([this] { m_drive->TankDriveVolts(0_V, 0_V); }, {})
//         // ,
//         // frc2::InstantCommand([this] {
//         //     m_arm->intake_arm_state = m_arm->UP_STATE;
//         //     m_intake->intake_state = m_intake->STOP_STATE;
//         //     m_shooter->shooter_state = m_shooter->SHOOT_STATE_H;
//         //     }),
//         // frc2::WaitCommand(1.5_s),
//         // frc2::InstantCommand([this] {
//         //     m_shooter->shooter_state = m_shooter->STOP_STATE_H;
//         //     })
//     );
// */


// namespace paths {
    
//     std::vector<frc::Trajectory> trajectories = {

//     };

//     void paths::SetContainer(RobotContainer container);

//     frc2::CommandBase* CrossInitiationLine();
//     frc2::CommandBase* OuttakePreloaded();
//     frc2::CommandBase* FiveBall();
//     frc2::CommandBase* FivePlusFiveBall();

//     frc2::CommandBase* GetSelected();



//     RobotContainer* m_container;

//     frc2::CommandBase* currentCommand;

//     frc::TrajectoryConfig* m_config;
//     std::string m_auton_name;

//     void SetTrajectoryConfig(frc::TrajectoryConfig* config);
//     void SetSelectedName(std::string name);
//     frc::Trajectory GetSelectedTrajectory(int index);

    
// }