// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/geometry/Pose2d.h>
#include <frc2/command/Commands.h>
#include <drive_distance.h>
#include <frc2/command/button/RobotModeTriggers.h>

RobotContainer::RobotContainer()
{
    ConfigureBindings();
}

void RobotContainer::ConfigureBindings()
{
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.SetDefaultCommand(

        // Drivetrain will execute this command periodically
        drivetrain.ApplyRequest([this]() -> auto&& {

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
        drivetrain.ApplyRequest([] {
            return swerve::requests::Idle{};
        }).IgnoringDisable(true)
    );

    joystick.A().WhileTrue(drivetrain.ApplyRequest([this]() -> auto&& { return brake; }));
    joystick.B().WhileTrue(drivetrain.ApplyRequest([this]() -> auto&& {
        return point.WithModuleDirection(frc::Rotation2d{-joystick.GetLeftY(), -joystick.GetLeftX()});
    }));

    joystick.X().OnTrue(
        DriveDistance(frc::Pose2d(frc::Translation2d(0_ft, 0_ft), frc::Rotation2d(0_deg)), &drivetrain).ToPtr()
        .AndThen(
            DriveDistance(frc::Pose2d(frc::Translation2d(3_ft, 0_ft), frc::Rotation2d(0_deg)), &drivetrain).ToPtr()
        )
        .AndThen(
            DriveDistance(frc::Pose2d(frc::Translation2d(3_ft, -3_ft), frc::Rotation2d(0_deg)), &drivetrain).ToPtr()
        )
        .AndThen(
            DriveDistance(frc::Pose2d(frc::Translation2d(0_ft, -3_ft), frc::Rotation2d(0_deg)), &drivetrain).ToPtr()
        )
        .AndThen(
            DriveDistance(frc::Pose2d(frc::Translation2d(0_ft, 0_ft), frc::Rotation2d(0_deg)), &drivetrain).ToPtr()
        )
    );

    joystick.Y().OnTrue(
        DriveDistance(frc::Pose2d(frc::Translation2d(0_ft, 0_ft), frc::Rotation2d(0_deg)), &drivetrain).ToPtr()
        .AndThen(
            DriveDistance(frc::Pose2d(frc::Translation2d(4_ft, -2_ft), frc::Rotation2d(0_deg)), &drivetrain).ToPtr()
        )
        .AndThen(
            DriveDistance(frc::Pose2d(frc::Translation2d(0_ft, -4_ft), frc::Rotation2d(0_deg)), &drivetrain).ToPtr()
        )
        .AndThen(
            DriveDistance(frc::Pose2d(frc::Translation2d(0_ft, 0_ft), frc::Rotation2d(0_deg)), &drivetrain).ToPtr()
        )
    );

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    (joystick.Back() && joystick.Y()).WhileTrue(drivetrain.SysIdDynamic(frc2::sysid::Direction::kForward));
    (joystick.Back() && joystick.X()).WhileTrue(drivetrain.SysIdDynamic(frc2::sysid::Direction::kReverse));
    (joystick.Start() && joystick.Y()).WhileTrue(drivetrain.SysIdQuasistatic(frc2::sysid::Direction::kForward));
    (joystick.Start() && joystick.X()).WhileTrue(drivetrain.SysIdQuasistatic(frc2::sysid::Direction::kReverse));

    // reset the field-centric heading on left bumper press
    joystick.LeftBumper().OnTrue(drivetrain.RunOnce([this] { drivetrain.TareEverything(); drivetrain.SeedFieldCentric();}));

    drivetrain.RegisterTelemetry([this](auto const &state) { logger.Telemeterize(state); });
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand()
{
    return frc2::cmd::Print("No autonomous command configured");
}
