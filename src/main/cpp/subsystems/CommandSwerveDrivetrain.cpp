#include "subsystems/CommandSwerveDrivetrain.h"
#include <frc/RobotController.h>

using namespace subsystems;

void CommandSwerveDrivetrain::Periodic()
{
    /*
     * Periodically try to apply the operator perspective.
     * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
     * This allows us to correct the perspective in case the robot code restarts mid-match.
     * Otherwise, only check and apply the operator perspective if the DS is disabled.
     * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
     */
    if (!m_hasAppliedOperatorPerspective || frc::DriverStation::IsDisabled()) {
        auto const allianceColor = frc::DriverStation::GetAlliance();
        if (allianceColor) {
            SetOperatorPerspectiveForward(
                *allianceColor == frc::DriverStation::Alliance::kRed
                    ? kRedAlliancePerspectiveRotation
                    : kBlueAlliancePerspectiveRotation
            );
            m_hasAppliedOperatorPerspective = true;
        }
    }
}

void CommandSwerveDrivetrain::StartSimThread()
{
    m_lastSimTime = utils::GetCurrentTime();
    m_simNotifier = std::make_unique<frc::Notifier>([this] {
        units::second_t const currentTime = utils::GetCurrentTime();
        auto const deltaTime = currentTime - m_lastSimTime;
        m_lastSimTime = currentTime;

        /* use the measured time delta, get battery voltage from WPILib */
        UpdateSimState(deltaTime, frc::RobotController::GetBatteryVoltage());
    });
    m_simNotifier->StartPeriodic(kSimLoopPeriod);
}

frc::Pose2d CommandSwerveDrivetrain::GetDrivetrainPoseEstimate()
{
    return GetState().Pose;
}

void CommandSwerveDrivetrain::DriveFieldCentric(units::meters_per_second_t xVel, units::meters_per_second_t yVel, units::radians_per_second_t zRot)
{
    SetControl(m_fieldCentricDrive.WithVelocityX(xVel).WithVelocityY(yVel).WithRotationalRate(zRot));
}

void CommandSwerveDrivetrain::FollowTrajectory(const choreo::SwerveSample& sample)
{
    // For now, fetch the estimation of where we are from just the drivetrain
    // TODO: In the future, fetch the position based off a fused odometry solution
    frc::Pose2d pose = GetDrivetrainPoseEstimate();

    // Calculate feedback velocities
    units::meters_per_second_t xFeedback{xController.Calculate(pose.X().value(), sample.x.value())};
    units::meters_per_second_t yFeedback{yController.Calculate(pose.Y().value(), sample.y.value())};
    units::radians_per_second_t headingFeedback{headingController.Calculate(pose.Rotation().Radians().value(), sample.heading.value())};

    // Apply feedback to robot
    DriveFieldCentric(xFeedback, yFeedback, headingFeedback);
}