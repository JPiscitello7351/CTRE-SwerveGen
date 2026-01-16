// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/CommandSwerveDrivetrain.h"
#include "subsystems/VisionSubsystem.h"
#include "commands/ServoToTag.h"

ServoToTag::ServoToTag(subsystems::CommandSwerveDrivetrain *pDrivetrain, subsystems::VisionSubsystem *pVision) 
: m_pDrive{pDrivetrain} 
, m_pVision{pVision}
{
  AddRequirements({pDrivetrain, pVision});
}

// Called when the command is initially scheduled.
void ServoToTag::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void ServoToTag::Execute() {
  using namespace subsystems;

  VisionData visd = m_pVision->GetLastData();
  units::radians_per_second_t speed;

  if (visd.hasTarget)
  {
    speed = units::radians_per_second_t{-1 * visd.tx.value() * 0.2};
  }
  else
  {
    speed = 0_rad_per_s;
  }

  m_pDrive->SetControl(rotVelReq.WithRotationalRate(speed));
}

// Called once the command ends or is interrupted.
void ServoToTag::End(bool interrupted) {
  // Passing with no other modifications sets x/y velocity to zero
  m_pDrive->SetControl(rotVelReq);
}

// Returns true when the command should end.
bool ServoToTag::IsFinished() {
  return false;
}
