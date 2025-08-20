// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/autonomous/trajectory_test.h"

#include "frc/smartdashboard/SmartDashboard.h"

#include "subsystems/CommandSwerveDrivetrain.h"

TrajectoryTest::TrajectoryTest(subsystems::CommandSwerveDrivetrain& drivetrain, std::optional<choreo::Trajectory<choreo::SwerveSample>> trajectory)
  : m_swerveDrivetrain{drivetrain},
    m_trajectory{trajectory} {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({&m_swerveDrivetrain});
}

// Called when the command is initially scheduled.
void TrajectoryTest::Initialize() {
  m_timer.Restart();
}

// Called repeatedly when this Command is scheduled to run
void TrajectoryTest::Execute() {
  static long alive = 0;

  if (m_trajectory.has_value())
  {
    if (auto sample = m_trajectory.value().SampleAt(m_timer.Get(), false))
    {
      frc::SmartDashboard::PutNumber("trajectory:alive", alive++);
      m_swerveDrivetrain.FollowTrajectory(sample.value());
    }
  }
}

// Called once the command ends or is interrupted.
void TrajectoryTest::End(bool interrupted) {}

// Returns true when the command should end.
bool TrajectoryTest::IsFinished() {
  return false;
}

std::string TrajectoryTest::GetName() const
{
  return "The best trajectory follower ever";
}

frc2::CommandPtr TrajectoryTest::GetCommand()
{
  return frc2::CommandPtr(std::move(*this));
}