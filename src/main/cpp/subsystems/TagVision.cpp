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
void TagVision::Periodic() {

    

}
void TagVision::setAllianceColor () {
        frc::DriverStation::Alliance allianceColor = frc::DriverStation::GetAlliance();
        if (allianceColor == frc::DriverStation::Alliance::kBlue) {
                tagLayout.SetOrigin(frc::AprilTagFieldLayout::OriginPosition::kBlueAllianceWallRightSide);
        } else {
                tagLayout.SetOrigin(frc::AprilTagFieldLayout::OriginPosition::kRedAllianceWallRightSide);
        }

        poseEstimator = new photonlib::PhotonPoseEstimator {
                tagLayout,
                photonlib::PoseStrategy::LOWEST_AMBIGUITY,
                std::move(cameraEstimate),
                robotToCamera
        };
}

void TagVision::updateOdometry() {
        std::optional<photonlib::EstimatedRobotPose> poseResult = poseEstimator->Update();

        if (poseResult.has_value()) {
                driveSubsystem->visionMeasurements(poseResult.value().estimatedPose.ToPose2d(), poseResult.value().timestamp);
        }
}

/*photonlib::EstimatedRobotPose TagVision::getEstimatedGlobalPose(frc::Pose2d prevEstimatedRobotPose) {
        poseEstimator.setReferencePose(prevEstimatedRobotPose);
        return poseEstimator.update();
}*/
