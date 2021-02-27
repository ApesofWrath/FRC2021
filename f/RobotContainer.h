#pragma once

#include <string>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/commands/Command.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/WaitUntilCommand.h>
#include <frc2/command/WaitCommand.h>

#include <frc2/command/PIDCommand.h>
#include <frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include <frc/trajectory/Trajectory.h>
#include <frc2/command/RamseteCommand.h>

#include "Drive/DriveBase.h"
#include <frc/Filesystem.h>
#include <frc/trajectory/TrajectoryUtil.h>
#include <wpi/Path.h>
#include <wpi/SmallString.h>

#include "Auton/AutonDrive.h"
#include "Shooter.h"
#include "Arm.h"
#include "Intake.h"

const std::string kAutoNameDefault = "Default";
const std::string kAutoNameCustom = "My Auto";
const std::string kAutoName_TestPath = "TestPath";
const std::string kAutoName_UnnamedPath = "UnnamedPath";
const std::string kAutoName_Unnamed = "Unnamed";

enum Auto {
  CROSS_INIT_LINE,
  SHOOT_PRELOAD,
  FIVE_BALL
};

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer(AutonDrive* drive, Shooter* shooter, Arm* arm, Intake* intake) : m_drive(drive), m_shooter(shooter), m_intake(intake), m_arm(arm) {};

  AutonDrive* m_drive;
  Shooter* m_shooter;
  Arm* m_arm;
  Intake* m_intake;

  void InitAutoChoices();

  frc2::Command* GetAutonomousCommand();
  frc::SendableChooser<Auto> m_chooser;
  
  Auto m_autoSelected;
 private:
  void ConfigureButtonBindings();



};