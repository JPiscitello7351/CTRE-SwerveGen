// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/CommandSwerveDrivetrain.h"
#include "subsystems/VisionSubsystem.h"
#include "commands/TranslateToTag.h"

TranslateToTag::TranslateToTag(subsystems::CommandSwerveDrivetrain *pDrive, subsystems::VisonSubsystem *pVision)
: m_pDrive{pDrive}
, m_pVision{pVision}
{
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({pDrive, pVision});
}

// Called when the command is initially scheduled.
void TranslateToTag::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void TranslateToTag::Execute() {
  using namespace subsystems;

  // Fetch the last data from the vision subsystem
  VisionData visd = m_pVision->GetLastData();
  units::meters_per_second_t xSpeed, ySpeed;

  // Remember your robot frames... y is robot "east" x is robot "north"
  if (visd.hasTarget)
  {
    // Based off of the degree tx/ty offset, calculate both the x and y velocities
    //   to pass to the drivetrain. This is essentially a crude "P" controller
    ySpeed = units::meters_per_second_t{-1 * visd.tx.value() * 0.05};
    xSpeed = units::meters_per_second_t{-1 * visd.ty.value() * 0.05};
  }
  else
  {
    // We don't have an active target, set speed to zero
    xSpeed = 0_mps;
    ySpeed = 0_mps;
  }

  // If the data is more than a second old... don't use it
  if (m_pVision->GetTimeSinceLastMeasure() > 1_s)
  {
    xSpeed = 0_mps;
    ySpeed = 0_mps;
  }

  // Send the request to the drivetrain to execute
  m_pDrive->SetControl(driveReq.WithVelocityY(ySpeed).WithVelocityX(xSpeed));
}

// Called once the command ends or is interrupted.
void TranslateToTag::End(bool interrupted) {
  // With no arguments, this will set velocity to zero
  m_pDrive->SetControl(driveReq);
}

// Returns true when the command should end.
bool TranslateToTag::IsFinished() {
  return false;
}
