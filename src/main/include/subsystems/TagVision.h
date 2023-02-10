// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
//#include <photonlib/RobotPoseEstimator.h>
#include <photonlib/PhotonCamera.h>
#include "photonlib/PhotonPoseEstimator.h"
//#include <photonlib/PhotonPoseEstimator.h>
#include <frc/geometry/Transform2d.h>
#include <frc/geometry/Transform3d.h>
#include <frc/geometry/Translation3d.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation3d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>

#include "Constants.h"
#include "subsystems/DriveSubsystem.h"

class TagVision : public frc2::SubsystemBase {
 public:

  TagVision(DriveSubsystem* driveSubsystem);


  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  //Set alliance layout so we are aware of were we are starting.
  void setAllianceColor(); 

  //Update vision position on the drive train.
  void updateOdometry();


  //photonlib::EstimatedRobotPose getEstimatedGlobalPose(frc::Pose2d prevEstimatedRobotPose);

  photonlib::PhotonCamera camera{ CameraNames::CAMERA_APRILTAG_FORWARD }; // Camera ID Number

 private:

  photonlib::PhotonCamera cameraEstimate{ CameraNames::CAMERA_APRILTAG_FORWARD }; // Camera ID Number
  //photonlib::PhotonPipelineResult cameraResult;
  frc::Transform3d robotToCamera = 
    frc::Transform3d(frc::Translation3d(Camerapos::CAMERA_APRILTAG_FORWARD_X, Camerapos::CAMERA_APRILTAG_FORWARD_Y, Camerapos::CAMERA_APRILTAG_FORWARD_Z), 
      frc::Rotation3d(0_rad, 0_rad, 0_rad));
  frc::AprilTagFieldLayout tagLayout{ frc::LoadAprilTagLayoutField(frc::AprilTagField::k2023ChargedUp) };
  photonlib::PhotonPoseEstimator* poseEstimator;//{ tagLayout, photonlib::PoseStrategy::CLOSEST_TO_REFERENCE_POSE, std::move(camera), robotToCamera };
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  DriveSubsystem* driveSubsystem; 
};
