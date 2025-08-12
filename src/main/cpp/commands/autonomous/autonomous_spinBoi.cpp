/// @copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/autonomous/autonomous_spinBoi.h"

#include <frc2/command/Commands.h>

AutonomousSpinBoi::AutonomousSpinBoi(subsystems::CommandSwerveDrivetrain& swerve, subsystems::SpinBoi& spinBoi)
    : m_swerve{swerve}
    , m_spinBoi{spinBoi}
 {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({&m_swerve, &m_spinBoi}); // Declare that this command requires the swerve drivetrain subsystem
}

// Called when the command is initially scheduled.
void AutonomousSpinBoi::Initialize() {
  
}

// Called repeatedly when this Command is scheduled to run
void AutonomousSpinBoi::Execute() {
  m_spinBoi.SetSpeed(1);

}

// Called once the command ends or is interrupted.
void AutonomousSpinBoi::End(bool interrupted) {
  m_spinBoi.SetSpeed(0);
}

// Returns true when the command should end.
bool AutonomousSpinBoi::IsFinished() {
  return false;
}

std::string AutonomousSpinBoi::GetName() const {
  return "02. SpinBoi";
}

frc2::CommandPtr AutonomousSpinBoi::GetCommand() {
  return frc2::CommandPtr(std::move(*this));
}
