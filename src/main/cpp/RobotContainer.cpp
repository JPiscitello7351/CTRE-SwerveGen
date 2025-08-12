// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/geometry/Pose2d.h>
#include <frc2/command/Commands.h>
#include <frc2/command/RunCommand.h>
#include <drive_distance.h>
#include <frc2/command/button/RobotModeTriggers.h>
#include "frc/smartdashboard/SmartDashboard.h"

// TODO: Debugging
#include "test_command.h"
#include "commands/autonomous/trajectory_test.h"
// END DEBUGGING

RobotContainer::RobotContainer()
: m_autoNothing(m_drivetrain)
, m_autoDriveForward(m_drivetrain)
, m_autoSpinBoi(m_drivetrain, m_spinBoi)
, m_autoSelector({  &m_autoNothing,
                    &m_autoDriveForward,
                    &m_autoSpinBoi}, &m_autoNothing) // Add more commands here as they are implemented
{
    ConfigureBindings();
}

void RobotContainer::ConfigureBindings()
{
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    m_drivetrain.SetDefaultCommand(

        // Drivetrain will execute this command periodically
        m_drivetrain.ApplyRequest([this]() -> auto&& {

            m_driveSpeedMultiplier = speeds::drive::driveSpeedMultiplier;   // Drive speed multiplier defined in constants.h
            m_turnSpeedMultiplier = speeds::drive::turnSpeedMultiplier;     // Turn speed multiplier defined in constants.h

            if(joystick.RightTrigger().Get()){ // Get the state of the right trigger and apply speed changes if bumper is pressed
                m_driveSpeedMultiplier = speeds::drive::turboDriveSpeedMultiplier;  // Turbo speed!!!
                m_turnSpeedMultiplier = speeds::drive::turboTurnSpeedMultiplier;    // Turbo turn rate!!!
            }
            else if(joystick.RightBumper().Get()){ // Get the state of the right bumper and apply speed changes if trigger is pressed
                m_driveSpeedMultiplier = speeds::drive::slowmoDriveSpeedMultiplier; // Slowmo speed...
                m_turnSpeedMultiplier = speeds::drive::slowmoTurnSpeedMultiplier;   // Slowmo turn rate...
            }

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

    joystick.X().OnTrue(
        DriveDistance(frc::Pose2d(frc::Translation2d(3_ft, 0_ft), frc::Rotation2d(0_deg)), &m_drivetrain).ToPtr()

        // .AndThen(
        //     DriveDistance(frc::Pose2d(frc::Translation2d(4_ft, -2_ft), frc::Rotation2d(330_deg)), &m_drivetrain).ToPtr()
        // )
        // .AndThen(
        //     DriveDistance(frc::Pose2d(frc::Translation2d(0_ft, -4_ft), frc::Rotation2d(210_deg)), &m_drivetrain).ToPtr()
        // )
        // .AndThen(
        //     DriveDistance(frc::Pose2d(frc::Translation2d(2_ft, 0_ft), frc::Rotation2d(270_deg)), &m_drivetrain).ToPtr()
        // )
        // .AndThen(
        //     DriveDistance(frc::Pose2d(frc::Translation2d(2_ft, -4_ft), frc::Rotation2d(90_deg)), &m_drivetrain).ToPtr()
        // )
        // .AndThen(
        //     DriveDistance(frc::Pose2d(frc::Translation2d(0_ft, 0_ft), frc::Rotation2d(210_deg)), &m_drivetrain).ToPtr()
        // )
    );

    joystick.Y().OnTrue(
        DriveDistance(frc::Pose2d(frc::Translation2d(0_ft, 0_ft), frc::Rotation2d(0_deg)), &m_drivetrain).ToPtr()
        .AndThen(
            DriveDistance(frc::Pose2d(frc::Translation2d(4_ft, -2_ft), frc::Rotation2d(0_deg)), &m_drivetrain).ToPtr()
        )
        .AndThen(
            DriveDistance(frc::Pose2d(frc::Translation2d(0_ft, -4_ft), frc::Rotation2d(0_deg)), &m_drivetrain).ToPtr()
        )
        .AndThen(
            DriveDistance(frc::Pose2d(frc::Translation2d(0_ft, 0_ft), frc::Rotation2d(0_deg)), &m_drivetrain).ToPtr()
        )
    );

    // Make spin boi spin (positive)
    joystick.POVRight().OnTrue(
        frc2::InstantCommand([this]() {m_spinBoi.SetSpeed(1);}, {&m_spinBoi}).ToPtr()
    ).OnFalse(
        frc2::InstantCommand([this]() {m_spinBoi.SetSpeed(0);}, {&m_spinBoi}).ToPtr()
    );

    // Make spin boi spin (negative)
    joystick.POVLeft().OnTrue(
        frc2::InstantCommand([this]() {m_spinBoi.SetSpeed(-1);}, {&m_spinBoi}).ToPtr()
    ).OnFalse(
        frc2::InstantCommand([this]() {m_spinBoi.SetSpeed(0);}, {&m_spinBoi}).ToPtr()
    );

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    (joystick.Back() && joystick.Y()).WhileTrue(m_drivetrain.SysIdDynamic(frc2::sysid::Direction::kForward));
    (joystick.Back() && joystick.X()).WhileTrue(m_drivetrain.SysIdDynamic(frc2::sysid::Direction::kReverse));
    (joystick.Start() && joystick.Y()).WhileTrue(m_drivetrain.SysIdQuasistatic(frc2::sysid::Direction::kForward));
    (joystick.Start() && joystick.X()).WhileTrue(m_drivetrain.SysIdQuasistatic(frc2::sysid::Direction::kReverse));

    // reset the field-centric heading on left bumper press
    joystick.LeftBumper().OnTrue(m_drivetrain.RunOnce([this] { m_drivetrain.TareEverything(); m_drivetrain.SeedFieldCentric();}));

    m_drivetrain.RegisterTelemetry([this](auto const &state) { logger.Telemeterize(state); });
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand()
{
    return m_autoSelector.GetSelectedCommand().value();
}
