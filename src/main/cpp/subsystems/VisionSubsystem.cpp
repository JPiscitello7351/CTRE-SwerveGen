// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <string>
#include <cmath>
#include <chrono>

#include "LimelightHelpers.h"
#include "subsystems/VisionSubsystem.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include "frc/geometry/Rotation3d.h"
#include "frc/geometry/Pose3d.h"

using namespace subsystems;

VisonSubsystem::VisonSubsystem(std::string limelightName)
: m_reading{true} // By default, periodic function will try to fetch data
, m_llName{limelightName}
{};

// This method will be called once per scheduler run
void VisonSubsystem::Periodic() {
    TryReadMeasurements();
} 

void
VisonSubsystem::TryReadMeasurements()
{
    using namespace LimelightHelpers;

    // Check if we want to read: we probably do, but allow user to turn off for
    //   performance.
    if (m_reading)
    {
        // Read in the vision data from networktables via helper functions
        // You could implement some logic here to not do this *every* periodic loop.
        // Note however, if you are running at even 60fps, that's essentially 60Hz, which is ~16ms
        //   which is still faster than the robot update loop of 50Hz (20ms). And typically, the 
        //   camera will be running at 120fps. So everytime this gets called, you'll have new data
        //   (at least through periodic, anyway)
        m_visionData.rxTimestamp = std::chrono::high_resolution_clock::now();
        // Array of raw fiducial markers with tx/ty and distance
        m_visionData.rawFiducials = getRawFiducials(m_llName);
        m_visionData.hasTarget = getTV(m_llName);
        // These next two data points are whatever the Limelight considers it's "target" this is controlled
        //   by both the ID filters in the config interface, as well as which target is "leftmost" (or whatever)
        //   prioritization scheme is used in the pipeline config
        m_visionData.tx = units::degree_t{getTX(m_llName)};
        m_visionData.ty = units::degree_t{getTY(m_llName)};
        // The relative pose (vector) of the tag, from the origin of the robot, in meters.
        m_visionData.targetPoseRobotSpace = toPose3D(getTargetPose_RobotSpace(m_llName));
        m_visionData.latency = units::millisecond_t{getLatency_Capture(m_llName) + getLatency_Pipeline(m_llName)};

        // Comment this out to disable printing to SmartDashboard
        dumpToSmartDashboard();
    }
}

VisionData VisonSubsystem::GetLastData() {return m_visionData;}

void VisonSubsystem::VisionOn() { m_reading = true; }

void VisonSubsystem::VisionOff() { m_reading = false; }

bool VisonSubsystem::IsVisionOn() { return m_reading; }

units::millisecond_t VisonSubsystem::GetTimeSinceLastMeasure()
{
    using namespace std::chrono;

    time_point<high_resolution_clock> now = high_resolution_clock::now();

    // duration cast to microseconds and divide in order to construct units::millisecond_t
    //   with fractional milliseconds for more accurate timestamp.
    return units::millisecond_t{duration_cast<microseconds>(now - m_visionData.rxTimestamp).count() / 1000};
}

// PRIVATE DEFINITIONS =========================================================

void
VisonSubsystem::dumpToSmartDashboard()
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