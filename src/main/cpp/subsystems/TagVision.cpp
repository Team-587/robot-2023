// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/TagVision.h"
#include <frc2/command/SubsystemBase.h>
//#include <photonlib/RobotPoseEstimator.h>
//#include <photonlib/EstimatedRobotPose>
#include <photonlib/PhotonCamera.h>
#include <photonlib/PhotonPoseEstimator.h>
#include <frc/geometry/Transform2d.h>
#include <frc/geometry/Transform3d.h>
#include <frc/geometry/Translation3d.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation3d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/apriltag/AprilTag.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/DriverStation.h>

TagVision::TagVision(DriveSubsystem* driveSubsystem): driveSubsystem(driveSubsystem) {

}

// This method will be called once per scheduler run
void TagVision::Periodic() {}

//sets the tag layout origin based on alliance color
void TagVision::setAllianceColor () {

        //get the alliance color
        frc::DriverStation::Alliance allianceColor = frc::DriverStation::GetAlliance();

        //set the origin
        if (allianceColor == frc::DriverStation::Alliance::kBlue) {
                tagLayout.SetOrigin(frc::AprilTagFieldLayout::OriginPosition::kBlueAllianceWallRightSide);
        } else {
                tagLayout.SetOrigin(frc::AprilTagFieldLayout::OriginPosition::kRedAllianceWallRightSide);
        }
        //init the pose estimator
        poseEstimator = new photonlib::PhotonPoseEstimator {
                tagLayout,
                photonlib::PoseStrategy::LOWEST_AMBIGUITY,
                std::move(cameraEstimate),
                robotToCamera
        };
}

//update the odometry on the drive subsystem
void TagVision::updateOdometry() {

        //update estimate and get the result
        std::optional<photonlib::EstimatedRobotPose> poseResult = poseEstimator->Update();

        //do we have a valid pose
        if (poseResult && poseResult.has_value()) {
                //update the odometry
                driveSubsystem->visionMeasurements(poseResult.value().estimatedPose.ToPose2d(), poseResult.value().timestamp);
        }
}

frc::AprilTagFieldLayout TagVision::getTagLayout() {
        return tagLayout;
}

/*photonlib::EstimatedRobotPose TagVision::getEstimatedGlobalPose(frc::Pose2d prevEstimatedRobotPose) {
        poseEstimator.setReferencePose(prevEstimatedRobotPose);
        return poseEstimator.update();
}*/
