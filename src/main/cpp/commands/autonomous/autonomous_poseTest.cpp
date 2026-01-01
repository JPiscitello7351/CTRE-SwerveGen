// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/autonomous/autonomous_poseTest.h"

#include "frc/smartdashboard/SmartDashboard.h"

#include "subsystems/CommandSwerveDrivetrain.h"

AutonomousPoseTest::AutonomousPoseTest(subsystems::CommandSwerveDrivetrain& drivetrain
, subsystems::SpinBoi& spinBoi
, std::optional<choreo::Trajectory<choreo::SwerveSample>> trajectory
, ChoreoEventManager &choreoEventManager)
: m_swerveDrivetrain{drivetrain}
, m_spinBoi{spinBoi}
, m_trajectory{trajectory}
, m_choreoEventRunner{choreoEventManager} 
{
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({&m_swerveDrivetrain, &m_spinBoi});
}

// Called when the command is initially scheduled.
void AutonomousPoseTest::Initialize() {
  m_timer.Restart();
  bool isRedAlliance = false;

  if(frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed){
    isRedAlliance = true;
  }

  if (m_trajectory.has_value())
  {
    m_swerveDrivetrain.ResetPose(m_trajectory.value().GetInitialPose(isRedAlliance).value());
  }
}

// Called repeatedly when this Command is scheduled to run
void AutonomousPoseTest::Execute() {
  static long alive = 0;

  if (m_trajectory.has_value())
  {
    m_choreoEventRunner.ScheduleActiveCommands(m_trajectory.value().events, m_timer.Get(), units::millisecond_t(100));
    
    if (auto sample = m_trajectory.value().SampleAt(m_timer.Get(), false))
    {
      frc::SmartDashboard::PutNumber("trajectory:alive", alive++);
      m_swerveDrivetrain.FollowTrajectory(sample.value());
    }
  }
}

// Called once the command ends or is interrupted.
void AutonomousPoseTest::End(bool interrupted) {}

// Returns true when the command should end.
bool AutonomousPoseTest::IsFinished() {
  return false;
}

std::string AutonomousPoseTest::GetName() const
{
  return "03. Pose Test";
}

frc2::CommandPtr AutonomousPoseTest::GetCommand()
{
  return frc2::CommandPtr(std::move(*this));
}
