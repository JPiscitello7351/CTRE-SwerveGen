// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/DriveToPose.h"

#include <frc/geometry/Pose2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <Eigen/Core>

#include "subsystems/CommandSwerveDrivetrain.h"

DriveToPose::DriveToPose(frc::Pose2d requestedPose, subsystems::CommandSwerveDrivetrain *drivetrain) : m_pSwerveDrive{drivetrain}, m_requestedPose(requestedPose)
{
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(m_pSwerveDrive);
}

// Called when the command is initially scheduled.
void DriveToPose::Initialize()
{
}

// Called repeatedly when this Command is scheduled to run
void DriveToPose::Execute() {
  static size_t alive = 0;
  // Speeds to drive at
  units::meters_per_second_t setSpeed = 0.35_mps;
  units::angular_velocity::radians_per_second_t rotationSetSpeed = 0.02_rad_per_s;

  // Calculate the individual x and y distance
  m_lastPose = m_pSwerveDrive->GetState().Pose;

  // Get x and y speeds, calculating unit vector components and multiply by set speed
  units::velocity::meters_per_second_t xSpeed = units::make_unit<units::meters_per_second_t>(m_xPidController.Calculate(m_lastPose.X().value(), m_requestedPose.X().value()));
  units::velocity::meters_per_second_t ySpeed = units::make_unit<units::meters_per_second_t>(m_yPidController.Calculate(m_lastPose.Y().value(), m_requestedPose.Y().value()));

  // Move *very slow* in the direction of the place we wanna go
  // In the future, there should be a function along the lines of m_pSwerveDrive.DriveWithVelocity(x, y, theta) inside of swerve subsystem

  m_pSwerveDrive->SetControl(m_fieldDriveOriented.WithVelocityX(xSpeed).WithVelocityY(ySpeed).WithTargetDirection(90_deg));

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
  // END
}

// Called once the command ends or is interrupted.
void DriveToPose::End(bool interrupted) {}

// Returns true when the command should end.
bool DriveToPose::IsFinished()
{
  // Find distance between where we are and where we want to be
  units::meter_t distance = m_requestedPose.Translation().Distance(m_lastPose.Translation());
  units::meter_t finTol = 0.1_m;
  units::angle::degree_t finAngTol = 5_deg;
  // Calculate distance from current pose to requested pose

  frc::SmartDashboard::PutNumber("DriveDisCmd:distance", distance.value());

  if (distance < finTol)
  {
    return true;
  }

  return false;
}
