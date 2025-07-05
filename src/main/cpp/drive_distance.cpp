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
  units::meter_t latTol = 0.08_m;
  // Speeds to drive at
  units::meters_per_second_t setSpeed = 0.35_mps;
  units::angular_velocity::radians_per_second_t rotationSetSpeed = 0.02_rad_per_s;
  // TODO: Should use a PID-like controller to do this

  // Get where the robot currently IS realative to Odomentry
  m_lastPose = m_pSwerveDrive->GetState().Pose;

  // Find the differece between us and our requested position (as a Transform2D)
  frc::Transform2d difference = m_requestedPose - m_lastPose;

  // Get heading vector and distance
  Eigen::Vector3d pathMatrix = difference.ToMatrix();
  units::length::meter_t vectorLength = difference.Translation().Norm();

  // Get x and y speeds, calculating unit vector components and multiply by set speed
  units::velocity::meters_per_second_t xSpeed = pathMatrix[0] / vectorLength.value() * setSpeed;
  units::velocity::meters_per_second_t ySpeed = pathMatrix[1] / vectorLength.value() * setSpeed;

  units::angular_velocity::radians_per_second_t rotSpeed = pathMatrix[2]*rotationSetSpeed;

  


/*
  // Negate speed if direction is opposite
  if (difference.X() > -latTol && difference.X() < latTol)
  {
    xSpeed = 0_mps;
  }
  else if (difference.X() < 0_m)
  {
    xSpeed *= -1;
  }

  if (difference.Y() > -latTol && difference.Y() < latTol)
  {
    ySpeed = 0_mps;
  }
  else if (difference.Y() < 0_m)
  {
    ySpeed *= -1;
  }

*/

  // Move *very slow* in the direction of the place we wanna go
  // In the future, there should be a function along the lines of m_pSwerveDrive.DriveWithVelocity(x, y, theta) inside of swerve subsystem
  m_pSwerveDrive->SetControl(m_fieldDrive.WithVelocityX(xSpeed).WithVelocityY(ySpeed).WithRotationalRate(rotSpeed));

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
  frc::SmartDashboard::PutNumber("DrivDisCmd:diffVec:angle", difference.Translation().Angle().Degrees().value());
  // END
}

// Called once the command ends or is interrupted.
void DriveDistance::End(bool interrupted) {}

// Returns true when the command should end.
bool DriveDistance::IsFinished()
{
  // Find the differece between us and our requested position (as a Transform2D)
  frc::Transform2d difference = m_requestedPose - m_lastPose;
  units::meter_t finTol = 0.1_m;
  // Calculate distance from current pose to requested pose

  if ((difference.X() > -finTol && difference.X() < finTol) && (difference.Y() > -finTol && difference.Y() < finTol))
  {
    return true;
  }

  return false;
}
