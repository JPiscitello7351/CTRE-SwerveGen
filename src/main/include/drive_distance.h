// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/geometry/Pose2d.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc/Timer.h>
#include "Constants.h"

// Include our subsystems to use in this command
#include "subsystems/CommandSwerveDrivetrain.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class DriveDistance
    : public frc2::CommandHelper<frc2::Command, DriveDistance> {
 public:
  /* You should consider using the more terse Command factories API instead
   * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
   */
  DriveDistance(frc::Pose2d requestedPose, subsystems::CommandSwerveDrivetrain* drivetrain);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;
private:
  subsystems::CommandSwerveDrivetrain* m_pSwerveDrive;
  frc::Pose2d m_lastPose;
  frc::Pose2d m_requestedPose;
  swerve::requests::FieldCentricFacingAngle m_fieldDriveOriented 
  = swerve::requests::FieldCentricFacingAngle{}
  .WithMaxAbsRotationalRate(speeds::drive::pathMaxTurnSpeed)
  .WithHeadingPID(20, 0, 0.1);
  frc::PIDController m_xPidController{2.5, 0.05, 0.0};
  frc::PIDController m_yPidController{2.5, 0.05, 0.0};
};
