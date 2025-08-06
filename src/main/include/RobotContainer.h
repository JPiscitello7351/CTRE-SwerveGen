// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "choreo/Choreo.h"
#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>
#include <drive_distance.h>
#include "subsystems/CommandSwerveDrivetrain.h"
#include "subsystems/SpinBoi.h"
#include "Telemetry.h"
#include "Constants.h"

#include "utils/auto_selector.h"
#include "commands/autonomous/autonomous_nothing.h"

class RobotContainer 
{
private:
    units::meters_per_second_t MaxSpeed = TunerConstants::kSpeedAt12Volts; // kSpeedAt12Volts desired top speed
    units::radians_per_second_t MaxAngularRate = 0.75_tps; // 3/4 of a rotation per second max angular velocity

    double m_driveSpeedMultiplier = 0.0;
    double m_turnSpeedMultiplier = 0.0;

    /* Setting up bindings for necessary control of the swerve drive platform */
    swerve::requests::FieldCentric drive = swerve::requests::FieldCentric{}
        .WithDeadband(MaxSpeed * 0.05).WithRotationalDeadband(MaxAngularRate * 0.05) // Add a 10% deadband
        .WithDriveRequestType(swerve::DriveRequestType::OpenLoopVoltage); // Use open-loop control for drive motors
    swerve::requests::SwerveDriveBrake brake{};
    swerve::requests::PointWheelsAt point{};

    /* Note: This must be constructed before the m_drivetrain, otherwise we need to
     *       define a destructor to un-register the telemetry from the m_drivetrain */
    Telemetry logger{MaxSpeed};

    frc2::CommandXboxController joystick{0};

    // Choreo timers
    frc::Timer m_timer;
    // Choreo trajectories
    std::optional<choreo::Trajectory<choreo::SwerveSample>> m_trajectory =
            choreo::Choreo::LoadTrajectory<choreo::SwerveSample>("myTrajectory");

public:
    subsystems::CommandSwerveDrivetrain m_drivetrain{TunerConstants::CreateDrivetrain()};
    subsystems::SpinBoi m_spinBoi{};

    RobotContainer();

    frc2::CommandPtr GetAutonomousCommand();

private:
    void ConfigureBindings();

    //Autonomous Commands
    AutonomousNothing m_autoNothing;
    
    AutoSelector m_autoSelector;
};
