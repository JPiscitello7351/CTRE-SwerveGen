// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/autonomous/testCommand.h"

testCommand::testCommand() {
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void testCommand::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void testCommand::Execute() {}

// Called once the command ends or is interrupted.
void testCommand::End(bool interrupted) {}

// Returns true when the command should end.
bool testCommand::IsFinished() {
  return false;
}
