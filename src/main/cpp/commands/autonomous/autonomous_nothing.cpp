/// @copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/autonomous/autonomous_nothing.h"

#include <frc2/command/Commands.h>

AutonomousNothing::AutonomousNothing(subsystems::CommandSwerveDrivetrain& swerve)
    : m_swerve{swerve}
 {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({&m_swerve}); // Declare that this command requires the swerve drivetrain subsystem
}

// Called when the command is initially scheduled.
void AutonomousNothing::Initialize() {
  
}

// Called repeatedly when this Command is scheduled to run
void AutonomousNothing::Execute() {
  m_swerve.SetControl(drive.WithVelocityX(0.5_mps) // Some forward movement
                     .WithVelocityY(0_mps) // No sideways movement
                     .WithRotationalRate(0_rad_per_s)); // No rotation


}

// Called once the command ends or is interrupted.
void AutonomousNothing::End(bool interrupted) {}

// Returns true when the command should end.
bool AutonomousNothing::IsFinished() {
  return false;
}

std::string AutonomousNothing::GetName() const {
  return "00. Do Nothing";
}

frc2::CommandPtr AutonomousNothing::GetCommand() {
  return frc2::CommandPtr(std::move(*this));
}
