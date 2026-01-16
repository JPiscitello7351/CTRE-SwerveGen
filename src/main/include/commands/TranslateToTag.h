// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "subsystems/CommandSwerveDrivetrain.h"
#include "subsystems/VisionSubsystem.h"
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class TranslateToTag
    : public frc2::CommandHelper<frc2::Command, TranslateToTag> {
 public:
  /* You should consider using the more terse Command factories API instead
   * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
   */
  TranslateToTag(subsystems::CommandSwerveDrivetrain *pDrive, subsystems::VisionSubsystem *pVision);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;
  private:
    subsystems::CommandSwerveDrivetrain *m_pDrive;
    subsystems::VisionSubsystem *m_pVision;
    swerve::requests::RobotCentric driveReq = swerve::requests::RobotCentric{}
        .WithDeadband(0.5_mps * 0.05).WithRotationalDeadband(0.75_tps * 0.05) // Add a 10% deadband
        .WithDriveRequestType(swerve::DriveRequestType::OpenLoopVoltage) // Use open-loop control for drive motors
        .WithVelocityX(0_mps)
        .WithVelocityY(0_mps);
};
