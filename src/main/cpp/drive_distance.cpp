// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "drive_distance.h"

#include "subsystems/CommandSwerveDrivetrain.h"

DriveDistance::DriveDistance(subsystems::CommandSwerveDrivetrain* drivetrain) : m_pSwerveDrive{drivetrain} {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(drivetrain);
}

// Called when the command is initially scheduled.
void DriveDistance::Initialize() {
}

// Called repeatedly when this Command is scheduled to run
void DriveDistance::Execute() {}

// Called once the command ends or is interrupted.
void DriveDistance::End(bool interrupted) {}

// Returns true when the command should end.
bool DriveDistance::IsFinished() {
  return false;
}
