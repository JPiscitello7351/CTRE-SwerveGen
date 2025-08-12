// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/CommandSwerveDrivetrain.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class TestCommand
    : public frc2::CommandHelper<frc2::Command, TestCommand> {
 public:
  /* You should consider using the more terse Command factories API instead
   * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
   */
  TestCommand(subsystems::CommandSwerveDrivetrain* pSwerveDrive);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  subsystems::CommandSwerveDrivetrain* m_pSwerveDrivetrain;
  swerve::requests::FieldCentric m_drive = swerve::requests::FieldCentric{}
      .WithDeadband(0.7_mps * 0.05).WithRotationalDeadband(0.5_rad_per_s * 0.05) // Add a 10% deadband
      .WithDriveRequestType(swerve::DriveRequestType::OpenLoopVoltage); // Use open-loop control for drive motors
};
