// #include "Auton.h"

// void paths::SetTrajectoryConfig(frc::TrajectoryConfig* config) {
//     paths::m_config = config;
// }

// void paths::BuildSelectedTrajectory() {

//     switch (paths::m_auton_name) {
//         case "Cross Initiation Line":
//             paths::currentCommand = paths
//             return
//         break;
//     }

// }

// void paths::SetContainer(RobotContainer container) {
//     paths::m_container = *container;
// }

// void paths::SetSelectedName(std::string name) {
//     paths::m_auton_name = name;
// }

// Trajectory paths::GetSelectedTrajectory(int index) {
//     if (index < paths::trajetories.size()) {
//         return paths::trajectories.at(index)
//     }
//     return frc::TrajectoryGenerator::GenerateTrajectory(
//         frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
//         {},
//         frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
//         *paths::m_config
//     )
// }
// frc2::CommandBase* CrossInitiationLine() {
//     std::vector<frc::Translation2d> points = {

//     };
//     auto trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
//         frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
//         {},
//         frc::Pose2d(2_m, 0_m, frc::Rotation2d(0_deg)),
//         *paths::m_config
//     )

//     frc2::RamseteCommand ramseteCommand(
//       trajectory, [this]() { return paths::m_container->m_drive->GetPose(); },
//       frc::RamseteController(K_RAMSETE_B,
//                               K_RAMSETE_ZETA),
//       frc::SimpleMotorFeedforward<units::meters>(
//           K_S, K_V, K_A),
//       K_DRIVE_KINEMATICS,
//       [this] { return paths::m_container->m_drive->GetWheelSpeeds(); },
//       frc2::PIDController(K_P_LEFT_VEL, 0, 0),
//       frc2::PIDController(K_P_RIGHT_VEL, 0, 0),
//       [this](auto left, auto right) { paths::m_container->m_drive->TankDriveVolts(left, right); },
//       {paths::m_container->m_drive});
    
//     return new frc2::SequentialCommandGroup(
//         std::move(ramseteCommand),
//         frc2::InstantCommand([] { paths::m_container->m_drive->TankDriveVolts(0_V, 0_V);})
//     );
// }
// frc2::CommandBase* OuttakePreloaded() {
//     return new frc2::SequentialCommandGroup();
// }
// frc2::CommandBase* FiveBall() {
//     return new frc2::SequentialCommandGroup();
// }
// frc2::CommandBase* FivePlusFiveBall() {
//     return new frc2::SequentialCommandGroup();
// }