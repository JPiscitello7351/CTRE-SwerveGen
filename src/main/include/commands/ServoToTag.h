// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/CommandSwerveDrivetrain.h"
#include "subsystems/VisionSubsystem.h"

/**
 * @brief A basic example command for VisonSubsystem and VisionData that rotates
 * a target to the center of the limelight crosshairs.
 * 
 * This example is very similar to TranslateToTag. In fact, the Execute() functions
 * are almost identical, with this command having one less speed to calculate. 
 * Read the header documentation for TranslateToTag first, then explore this
 * command if you need more information.
 * 
 * @author Jacob S.
 */
class ServoToTag
    : public frc2::CommandHelper<frc2::Command, ServoToTag> {
 public:
  /* You should consider using the more terse Command factories API instead
   * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
   */
  ServoToTag(subsystems::CommandSwerveDrivetrain *pDrivetrain, subsystems::VisonSubsystem *pVision);


  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;
  private:
    subsystems::CommandSwerveDrivetrain *m_pDrive;   
    subsystems::VisonSubsystem *m_pVision;   
    /// @brief Rotational velocity request (default 0 x/y velocity)
    swerve::requests::FieldCentric rotVelReq = swerve::requests::FieldCentric{}
        .WithDeadband(1_mps * 0.05).WithRotationalDeadband(0.75_tps * 0.05) // Add a 10% deadband
        .WithDriveRequestType(swerve::DriveRequestType::OpenLoopVoltage) // Use open-loop control for drive motors
        .WithVelocityX(0_mps)
        .WithVelocityY(0_mps);
};
