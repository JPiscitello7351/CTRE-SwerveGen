// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>

#include <frc2/command/SubsystemBase.h>
#include <frc/geometry/Pose3d.h>
#include "units/length.h"

#include "LimelightHelpers.h"

namespace subsystems {
  class VisionData {
    public:
      bool hasTargets;
      units::degree_t tx;
      units::degree_t ty;

      // For more information on this data, see LimelightHelpers.h
      std::vector<LimelightHelpers::RawFiducial> rawFiducials;

      units::millisecond_t latency;

      frc::Pose3d targetPoseRobotSpace;
  };

  class VisionSubsystem : public frc2::SubsystemBase {
  public:
    VisionSubsystem(std::string limelightName);

    /**
     * Will be called periodically whenever the CommandScheduler runs.
     */
    void Periodic() override;

    VisionData GetLastData();

    /**
     * @brief Turn vision measurements "on". This will set m_ready and start the flow of data
     * 
     */
    void VisionOn();

    /**
     * @brief Opposite of VisionOn()
     * 
     */
    void VisionOff();

    /**
     * @brief Return the state of the vision measuring system
     * 
     * @return true The camera is collecting data and updating poses and targets.
     * @return false The camera is stopped, and not collecting data.
     */
    bool IsVisionOn();

    /**
     * @brief If the vision is enabled, try to read and parse camera targets from
     * the limelight.
     * 
     */
    void TryReadMeasurements();

  private:
  /**
    * @brief Dump provided results to smartdashboard for debugging
    * 
    * @param rslts Object reference to limelight results to dump
    */
    void dumpToSmartDashboard();


    /**
     * @brief Returns the distance (in meters) to the given target
     * 
     * @param targetInRobotSpace Target, must be a vector<double> of size 6
     * @return std::optional<units::meter_t> Distance to provided target, in meters, otherwise nullopt
     */
    std::optional<units::meter_t> calculateDistanceToTarget(std::vector<double> targetInRobotSpace);

    bool m_reading; ///< If the subsystem will read data during Periodic()
    VisionData m_visionData = {};
    std::string m_llName; ///< Name of limelight camera (must match limelight configuration in web dashboard)

    // Components (e.g. motor controllers and sensors) should generally be
    // declared private and exposed only through public methods.
  };
}
