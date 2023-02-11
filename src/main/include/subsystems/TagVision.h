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

  frc::AprilTagFieldLayout getTagLayout();
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  //Set alliance layout so we are aware of were we are starting.
  void setAllianceColor(); 

  //Update vision position on the drive train.
  void updateOdometry();


  //photonlib::EstimatedRobotPose getEstimatedGlobalPose(frc::Pose2d prevEstimatedRobotPose);

  //
  //AK use the vision container for this???
  //
  photonlib::PhotonCamera camera{ CameraNames::CAMERA_APRILTAG_FORWARD }; // Camera ID Number

 private:

  //Camera for the pose estimator
  photonlib::PhotonCamera cameraEstimate{ CameraNames::CAMERA_APRILTAG_FORWARD }; // Camera ID Number
  
  //Camera location on the robot
  frc::Transform3d robotToCamera = 
    frc::Transform3d(frc::Translation3d(Camerapos::CAMERA_APRILTAG_FORWARD_X, Camerapos::CAMERA_APRILTAG_FORWARD_Y, Camerapos::CAMERA_APRILTAG_FORWARD_Z), 
      frc::Rotation3d(0_rad, 0_rad, 0_rad));

  //The april tag layout for 2023
  frc::AprilTagFieldLayout tagLayout{ frc::LoadAprilTagLayoutField(frc::AprilTagField::k2023ChargedUp) };

  //This is the pose estimator
  photonlib::PhotonPoseEstimator* poseEstimator;
  
  //the drive subsystem to update the odometry
  DriveSubsystem* driveSubsystem; 
};
