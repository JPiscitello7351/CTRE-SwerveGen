/// @copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include <string>

#include "commands/autonomous/autonomous_command.h"
#include "subsystems/CommandSwerveDrivetrain.h"
#include "subsystems/SpinBoi.h"

class AutonomousSpinBoi
    : public frc2::CommandHelper<frc2::Command, AutonomousSpinBoi>
    , public AutonomousCommand {
 public:
  explicit AutonomousSpinBoi(subsystems::CommandSwerveDrivetrain& swerve, subsystems::SpinBoi& spinBoi);

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
  subsystems::SpinBoi& m_spinBoi;
};
