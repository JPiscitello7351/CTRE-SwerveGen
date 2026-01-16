/**
 * This vision system consumes data from a Limelight camera attached to the network. 
 * You, the user, configure the name of this limelight to match what is configured
 * in the limelight dashboard.
 * Data is consumed via networktables and the LimelightHelpers.h header-only library
 * provided by Limelight themselves. See the below github repo for details:
 * https://github.com/LimelightVision/limelightlib-wpicpp/tree/main
 */

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <string>
#include <cmath>

#include "LimelightHelpers.h"
#include "subsystems/VisionSubsystem.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include "frc/geometry/Rotation3d.h"
#include "frc/geometry/Pose3d.h"

using namespace subsystems;

VisionSubsystem::VisionSubsystem(std::string limelightName)
: m_reading{true} // By default, periodic function will try to fetch data
, m_llName{limelightName}
{};

// This method will be called once per scheduler run
void VisionSubsystem::Periodic() {
    TryReadMeasurements();
} 

void
VisionSubsystem::TryReadMeasurements()
{
    using namespace LimelightHelpers;

    // Check if we want to read: we probably do, but allow user to turn off for
    //   performance.
    if (m_reading)
    {
        // Read in the vision data from networktables, and parse the JSON
        // You could implement some logic here to not do this *every* periodic loop.
        // Note however, if you are running at even 60fps, that's essentially 60Hz, which is ~16ms
        //   which is still faster than the robot update loop of 50Hz (20ms)


        m_visionData.rawFiducials = getRawFiducials(m_llName);
        m_visionData.hasTargets = m_visionData.rawFiducials.size() > 0;
        m_visionData.tx = units::degree_t{getTX(m_llName)};
        m_visionData.ty = units::degree_t{getTY(m_llName)};
        m_visionData.targetPoseRobotSpace = toPose3D(getTargetPose_RobotSpace(m_llName));
        m_visionData.latency = units::millisecond_t{getLatency_Capture(m_llName) + getLatency_Pipeline(m_llName)};

        // Comment this out to disable printing to SmartDashboard
        dumpToSmartDashboard();
    }
}


VisionData VisionSubsystem::GetLastData() {return m_visionData;}

void VisionSubsystem::VisionOn() { m_reading = true; }

void VisionSubsystem::VisionOff() { m_reading = false; }

bool VisionSubsystem::IsVisionOn() { return m_reading; }

// PRIVATE DEFINITIONS =========================================================

void
VisionSubsystem::dumpToSmartDashboard()
{
    std::string prefix{};
    std::string val{};
    frc::SmartDashboard::PutNumber("Vision/latency", m_visionData.latency.value());
    frc::SmartDashboard::PutNumber("Vision/tx-deg", m_visionData.tx.value());
    frc::SmartDashboard::PutNumber("Vision/ty-deg", m_visionData.ty.value());

    frc::SmartDashboard::PutNumberArray(
        "Vision/target-robotspace",
        std::vector{
            m_visionData.targetPoseRobotSpace.X().value(),
            m_visionData.targetPoseRobotSpace.Y().value(),
            m_visionData.targetPoseRobotSpace.Z().value()
        }
    );

    std::vector<LimelightHelpers::RawFiducial> &fids = m_visionData.rawFiducials;

    for (size_t i = 0; i < fids.size(); i++)
    {
        prefix = "Vision/tag[" + std::to_string(fids[i].id) + "]";

        frc::SmartDashboard::PutNumber(prefix + "/distToCam_m", fids[i].distToCamera);
        frc::SmartDashboard::PutNumber(prefix + "/distToRobot_m", fids[i].distToRobot);
        frc::SmartDashboard::PutNumber(prefix + "/ambiguity", fids[i].ambiguity);
    }
}