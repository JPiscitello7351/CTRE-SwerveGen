// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "drive_distance.h"

#include <frc/geometry/Pose2d.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "subsystems/CommandSwerveDrivetrain.h"

DriveDistance::DriveDistance(frc::Pose2d requestedPose, subsystems::CommandSwerveDrivetrain *drivetrain) : m_pSwerveDrive{drivetrain}, m_requestedPose{requestedPose}
{
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(m_pSwerveDrive);
}

// Called when the command is initially scheduled.
void DriveDistance::Initialize()
{
}

// Called repeatedly when this Command is scheduled to run
void DriveDistance::Execute() {
  static size_t alive = 0;
  // Speeds to drive at
  units::meters_per_second_t xSpeed = 0.5_mps;
  units::meters_per_second_t ySpeed = xSpeed;
  // TODO: Should use a PID-like controller to do this

  // Get where the robot currently IS realative to Odomentry
  m_lastPose = m_pSwerveDrive->GetState().Pose;

  // Find the differece between us and our requested position (as a Transform2D)
  frc::Transform2d difference = m_requestedPose - m_lastPose;

  // Negate speed if direction is opposite
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

  // DEBUGGING: START
  // Last position
  frc::SmartDashboard::PutNumber("DrivDisCmd:alive", alive++);
  frc::SmartDashboard::PutNumber("DrivDisCmd:lastPos:x", m_lastPose.X().value());
  frc::SmartDashboard::PutNumber("DrivDisCmd:lastPos:y", m_lastPose.Y().value());
  // Speeds
  frc::SmartDashboard::PutNumber("DrivDisCmd:xspeed", xSpeed.value());
  frc::SmartDashboard::PutNumber("DrivDisCmd:yspeed", ySpeed.value());
  // Requested position
  frc::SmartDashboard::PutNumber("DrivDisCmd:reqPose:x", m_requestedPose.X().value());
  frc::SmartDashboard::PutNumber("DrivDisCmd:reqPose:y", m_requestedPose.Y().value());
  // Difference vector
  frc::SmartDashboard::PutNumber("DrivDisCmd:diffVec:x", difference.X().value());
  frc::SmartDashboard::PutNumber("DrivDisCmd:diffVec:y", difference.X().value());
  // END
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

  // DEBUGGING: START
  frc::SmartDashboard::PutNumber("DrivDisCmd:distance", distance.value());
  // END

  return false;
}
