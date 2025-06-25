// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "drive_distance.h"

#include <frc/geometry/Pose2d.h>

#include "subsystems/CommandSwerveDrivetrain.h"

DriveDistance::DriveDistance(frc::Pose2d requestedPose, subsystems::CommandSwerveDrivetrain *drivetrain) : m_pSwerveDrive{drivetrain}, m_requestedPose{requestedPose}
{
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(drivetrain);
}

// Called when the command is initially scheduled.
void DriveDistance::Initialize()
{
}

// Called repeatedly when this Command is scheduled to run
void DriveDistance::Execute() {
  // Speeds to drive at
  units::meters_per_second_t xSpeed = 0.10_ms;
  units::meters_per_second_t ySpeed = xSpeed;
  // TODO: Should use a PID-like controller to do this

  // Get where the robot currently IS realative to Odomentry (0,0)
  m_lastPose = m_pSwerveDrive->GetState().Pose;

  // Find the differece between us and our requested position (as a Transform2D)
  frc::Transform2d difference = m_requestedPose - m_lastPose;

  // Negate speed if direction is different
  if (difference.X() < 0_m)
  {
    xSpeed *= -1;
  }

  if (difference.Y() < 0_m)
  {
    ySpeed *= -1;
  }

  // Move *very slow* in the direction of the place we wanna go
  // In the future, there should be a function along the lines of m_pSwerveDrive.DriveWithVelocity(x, y, theta) inside of swerve subsystem
  m_pSwerveDrive->ApplyRequest([this, xSpeed, ySpeed]() -> auto&& { return m_fieldDrive.WithVelocityX(xSpeed).WithVelocityY(ySpeed).WithRotationalRate(0_rad_per_s); });
}

// Called once the command ends or is interrupted.
void DriveDistance::End(bool interrupted) {}

// Returns true when the command should end.
bool DriveDistance::IsFinished()
{
  // Calculate distance from current pose to requested pose
  units::meter_t distance = m_requestedPose.Translation().Distance(m_lastPose.Translation());

  if (distance < 0.1_m)
  {
    return true;
  }

  return false;
}
