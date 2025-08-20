/// @copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/autonomous/autonomous_driveForward.h"

#include <frc2/command/Commands.h>

AutonomousDriveForward::AutonomousDriveForward(subsystems::CommandSwerveDrivetrain& swerve)
    : m_swerve{swerve}
 {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({&m_swerve}); // Declare that this command requires the swerve drivetrain subsystem
}

// Called when the command is initially scheduled.
void AutonomousDriveForward::Initialize() {
  
}

// Called repeatedly when this Command is scheduled to run
void AutonomousDriveForward::Execute() {
  m_swerve.SetControl(drive.WithVelocityX(0.5_mps) // Some forward movement
                     .WithVelocityY(0_mps) // No sideways movement
                     .WithRotationalRate(0_rad_per_s)); // No rotation


}

// Called once the command ends or is interrupted.
void AutonomousDriveForward::End(bool interrupted) {}

// Returns true when the command should end.
bool AutonomousDriveForward::IsFinished() {
  return false;
}

std::string AutonomousDriveForward::GetName() const {
  return "01. Drive Forward";
}

frc2::CommandPtr AutonomousDriveForward::GetCommand() {
  return frc2::CommandPtr(std::move(*this));
}
