/// @copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include <string>

#include "commands/autonomous/autonomous_command.h"
#include "subsystems/CommandSwerveDrivetrain.h"
#include "generated/TunerConstants.h"

class AutonomousNothing
    : public frc2::CommandHelper<frc2::Command, AutonomousNothing>
    , public AutonomousCommand {
 public:
  explicit AutonomousNothing(subsystems::CommandSwerveDrivetrain& swerve);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  /**
   * @copydoc AutonomousCommand::GetName()
   */
  std::string GetName() const final;
  /**
   * @copydoc AutonomousCommand::GetCommand()
   */
  frc2::CommandPtr GetCommand() final;

 private:
  subsystems::CommandSwerveDrivetrain& m_swerve;
  swerve::requests::FieldCentric drive = swerve::requests::FieldCentric{}
        .WithDeadband(TunerConstants::kSpeedAt12Volts * 0.05).WithRotationalDeadband(0.75_tps * 0.05) // Add a 10% deadband
        .WithDriveRequestType(swerve::DriveRequestType::OpenLoopVoltage); // Use open-loop control for drive motors
};
