// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/WaitCommand.h>
#include <frc/geometry/Pose2d.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/StateSpaceUtil.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/ShuffleboardTab.h>
#include <frc/DriverStation.h>
#include <iostream>

#include <wpi/array.h>

#include <photonlib/PhotonCamera.h>

#include "Constants.h"
#include "DriveSubsystem.h"

//
//
//change heading to gyroscopic
//getmodulestates
//fix the units
//
//



class PoseEstimatorSubsystem : public frc2::SubsystemBase {
  public:

    PoseEstimatorSubsystem(photonlib::PhotonCamera *pCamera, DriveSubsystem *pDriveSubsystem);
    ~PoseEstimatorSubsystem();

    void Periodic() override;

    void Update();

    std::string GetFomattedPose();
    frc::Pose2d GetCurrentPose();
    void SetCurrentPose(frc::Pose2d newPose);
    void ResetFieldPosition();
    frc::AprilTagFieldLayout* GetLayout() { return &tagLayout; };

  private:

    //delay the start
    frc2::WaitCommand m_WaitCommand{2.0_s};
  
    //camera configured for april tags
    photonlib::PhotonCamera *m_pCamera;

    //drive subsystem for pose estimation
    DriveSubsystem *m_pDriveSubsystem;

    // Physical location of the camera on the robot, relative to the center of the
    // robot.
    const frc::Transform2d CAMERA_TO_ROBOT = 
      frc::Transform2d(
        frc::Translation2d(
          Camerapos::CAMERA_APRILTAG_FORWARD_X, 
          Camerapos::CAMERA_APRILTAG_FORWARD_Y), 
        frc::Rotation2d(0_deg));

    //The april tag layout for 2023
    frc::AprilTagFieldLayout tagLayout{ frc::LoadAprilTagLayoutField(frc::AprilTagField::k2023ChargedUp) }; 
    
    frc::SwerveDrivePoseEstimator<4> *m_pPoseEstimator = NULL;

    frc::Field2d field2d;

};
