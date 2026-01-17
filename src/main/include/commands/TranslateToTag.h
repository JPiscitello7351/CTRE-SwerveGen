// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "subsystems/CommandSwerveDrivetrain.h"
#include "subsystems/VisionSubsystem.h"
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

/**
 * @brief Keeps a tag centered in Limelight's crosshairs by translating the
 * robot in robot's x/y direction (robot centric, not field centric!)
 * 
 * This command demonstrates how to use the subsystems::VisonSubsystem and subsystems::VisionData to
 * generate velocities for the robot to center to a tag. The point the robot
 * centers on is currently 0 degrees tx and ty, but this could be easily
 * easily changed in the pipeline config by adjusting the crosshair, or in code
 * by subtracting an offset from the tx/ty values.
 * 
 * On end, the command will set the drivetrain to zero x/y velocity.
 * 
 * When a tag is not seen, the velocities will be set to zero.
 * 
 * There is no end condiditon, it's useful to bind this command to a button
 * with WhileTrue() trigger. See RobotContainer.cpp for an example of this for
 * this command.
 * 
 * @author Jacob S.
 * 
 */
class TranslateToTag
    : public frc2::CommandHelper<frc2::Command, TranslateToTag> {
 public:
  /* You should consider using the more terse Command factories API instead
   * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
   */
  TranslateToTag(subsystems::CommandSwerveDrivetrain *pDrive, subsystems::VisonSubsystem *pVision);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;
  private:
    subsystems::CommandSwerveDrivetrain *m_pDrive;
    subsystems::VisonSubsystem *m_pVision;
    /// @brief Basic swerve drive request from CTRE Phoenix 6 library. x/y velocities 0 by default
    swerve::requests::RobotCentric driveReq = swerve::requests::RobotCentric{}
        .WithDeadband(0.5_mps * 0.05).WithRotationalDeadband(0.75_tps * 0.05) // Add a 10% deadband
        .WithDriveRequestType(swerve::DriveRequestType::OpenLoopVoltage) // Use open-loop control for drive motors
        .WithVelocityX(0_mps)
        .WithVelocityY(0_mps);
};
