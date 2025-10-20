// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/geometry/Pose2d.h>
#include <frc2/command/Commands.h>
#include <frc2/command/RunCommand.h>
#include <commands/DriveToPose.h>
#include <frc2/command/button/RobotModeTriggers.h>
#include "frc/smartdashboard/SmartDashboard.h"

RobotContainer::RobotContainer()
: m_autoNothing(m_drivetrain)
, m_trajectoryTest(m_drivetrain, m_trajectory, m_choreoEventManager)
, m_autoSelector({  &m_autoNothing,
                    &m_trajectoryTest}, &m_autoNothing) // Add more commands here as they are implemented
, m_choreoEventManager()
{
    ConfigureBindings();
}

void RobotContainer::ConfigureBindings()
{
    //Configure event manager command mapping
    m_choreoEventManager.AddKey("runIntake", PrintStuff("Running intake!").ToPtr());
    m_choreoEventManager.AddKey("intakeDown", PrintStuff("Intake down!").ToPtr());
    m_choreoEventManager.AddKey("intakeUp", PrintStuff("Intake up!").ToPtr());

    
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    m_drivetrain.SetDefaultCommand(

        // Drivetrain will execute this command periodically
        m_drivetrain.ApplyRequest([this]() -> auto&& {

            m_driveSpeedMultiplier = speeds::drive::driveSpeedMultiplier;   // Drive speed multiplier defined in constants.h
            m_turnSpeedMultiplier = speeds::drive::turnSpeedMultiplier;     // Turn speed multiplier defined in constants.h

            return drive.WithVelocityX(-joystick.GetLeftY() * MaxSpeed * m_driveSpeedMultiplier)        // Drive forward with negative Y (forward)
                .WithVelocityY(-joystick.GetLeftX() * MaxSpeed * m_driveSpeedMultiplier)                // Drive left with negative X (left)
                .WithRotationalRate(-joystick.GetRightX() * MaxAngularRate * m_turnSpeedMultiplier);    // Drive counterclockwise with negative X (left)
        })
    );

    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    frc2::RobotModeTriggers::Disabled().WhileTrue(
        m_drivetrain.ApplyRequest([] {
            return swerve::requests::Idle{};
        }).IgnoringDisable(true)
    );

    joystick.A().WhileTrue(m_drivetrain.ApplyRequest([this]() -> auto&& { return brake; }));
    joystick.B().WhileTrue(m_drivetrain.ApplyRequest([this]() -> auto&& {
        return point.WithModuleDirection(frc::Rotation2d{-joystick.GetLeftY(), -joystick.GetLeftX()});
    }));

    // reset the field-centric heading on left bumper press
    joystick.LeftBumper().OnTrue(m_drivetrain.RunOnce([this] {m_drivetrain.SeedFieldCentric();}));

    m_drivetrain.RegisterTelemetry([this](auto const &state) { logger.Telemeterize(state); });
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand()
{
    return m_autoSelector.GetSelectedCommand().value();
}
