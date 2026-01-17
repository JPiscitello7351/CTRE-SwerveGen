/**
 * @file VisonSubsystem.h
 * @author Jacob Simeone
 * @brief Simple limelight camera vision subsystem primarily for detecting AprilTags
 * @date 2026-01-16
 * 
 */

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>
#include <chrono>

#include <frc2/command/SubsystemBase.h>
#include <frc/geometry/Pose3d.h>
#include "units/length.h"

#include "LimelightHelpers.h"

namespace subsystems {
  /**
   * @brief Holds vision data produced by the VisonSubsystem
   * 
   */
  class VisionData {
    public:
      bool hasTarget; ///< True if a target (with tx/ty values was detected)
      units::degree_t tx; ///< target x-offset from center crosshair of limelight
      units::degree_t ty; ///< Same as tx for y offset

      std::vector<LimelightHelpers::RawFiducial> rawFiducials; ///< See LimlighhtHelpers.h for more info

      units::millisecond_t latency; ///< Combined pipeline and capture latency
      std::chrono::time_point<std::chrono::high_resolution_clock> rxTimestamp; ///< Time this data class was constructed

      frc::Pose3d targetPoseRobotSpace; ///< The target pose, as a vector from the robot's origin
  };

  /**
   * @brief Example vision subystem for a single limelight camera, optimized for Apriltags.
   * 
   * This vision system consumes data from a Limelight camera attached to the network. 
   * You, the user, configure the name of this limelight to match what is configured
   * in the limelight dashboard.
   * Data is consumed via networktables and the LimelightHelpers.h header-only library
   * provided by Limelight themselves.
   * 
   * Data is polled by the periodic function, and reading can be switched off through
   * the VisionOn/Off functions. Data is returned as a VisionData object written
   * specifically for this subsystem, and can be viewed/edited in the header of this
   * subsystem. 
   * 
   * Some improvements to this subsystem could be pipeline switching, ID filtering,
   * or even some basic pose filtering (but I would almost wonder if a dedicated
   * odometry class would be more appropriate for that). Further, there could be
   * some more limelight control via LEDs or other hardware features. Note that 
   * some other code somewhere else in the project should be responsible for
   * converting tx/ty values to a robot heading for advanced control schemes.
   * (Like a PID heading controller that lives in the drivetrain subsystem)
   * 
   * It may also be beneficial to either have multiple instances of this subsystem
   * and rename this to "limelight" subsystem and make another wrapper for this 
   * that holds all cameras in one "vision" subsystem or, add some capabilities
   * to this existing code to handle multiple limelight cameras (should be pretty
   * easy, and you could use the limelight name as the unique key for any data)
   * 
   * **RELATED RESOURCES**:
   * 
   * Limelight 4 Quick-Start: https://docs.limelightvision.io/docs/docs-limelight/getting-started/limelight-4
   * 
   * Apriltag Pipelines: https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltags
   * 
   * Programming Guide for Limelight: https://docs.limelightvision.io/docs/docs-limelight/apis/limelight-lib
   * 
   * Github repo for limelight helpers: https://github.com/LimelightVision/limelightlib-wpicpp/tree/main
   * 
   * @author Jacob S.
   */
  class VisonSubsystem : public frc2::SubsystemBase {
  public:
    VisonSubsystem(std::string limelightName);

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

    /**
     * @brief Calculate the time since the vision data was last updated. The
     * age of the vision data returned by GetLastData()
     * 
     * @return units::millisecond_t Time since last measurement taken
     */
    units::millisecond_t GetTimeSinceLastMeasure();

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
