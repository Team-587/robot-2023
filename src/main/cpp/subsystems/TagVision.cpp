// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/TagVision.h"
#include <frc2/command/SubsystemBase.h>
//#include <photonlib/RobotPoseEstimator.h>
//#include <photonlib/EstimatedRobotPose>
#include <photonlib/PhotonCamera.h>
//#include <photonlib/PhotonPoseEstimator.h>
//#include <photonlib/RobotPoseEstimator.h>
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

        camera.SetPipelineIndex(2);
        cameraEstimate1->SetPipelineIndex(2);
        cameraEstimate2->SetPipelineIndex(0);
}

// This method will be called once per scheduler run
void TagVision::Periodic() {}

//sets the tag layout origin based on alliance color
void TagVision::setAllianceColor () {

        //get the alliance color
        frc::DriverStation::Alliance allianceColor = frc::DriverStation::GetAlliance();

        //set the origin
        if (allianceColor == frc::DriverStation::Alliance::kBlue) {
                tagLayout->SetOrigin(frc::AprilTagFieldLayout::OriginPosition::kBlueAllianceWallRightSide);
        } else {
                tagLayout->SetOrigin(frc::AprilTagFieldLayout::OriginPosition::kRedAllianceWallRightSide);
        }

        std::vector<std::pair<std::shared_ptr<photonlib::PhotonCamera>, frc::Transform3d>> cameras;
        cameras.push_back(std::make_pair(cameraEstimate1, robotToCamera1));
        cameras.push_back(std::make_pair(cameraEstimate2, robotToCamera2));
        //init the pose estimator
        poseEstimator = new photonlib::RobotPoseEstimator  {
                tagLayout,
                //photonlib::PoseStrategy::LOWEST_AMBIGUITY,
                //photonlib::PoseStrategy::AVERAGE_BEST_TARGETS,
                photonlib::PoseStrategy::CLOSEST_TO_REFERENCE_POSE,
                cameras

                
        };
}

//update the odometry on the drive subsystem
void TagVision::updateOdometry(frc::Pose2d prevEstimatedRobotPose) {

        //set the reference pose and last pose
        poseEstimator->SetReferencePose(frc::Pose3d(prevEstimatedRobotPose));
        poseEstimator->SetLastPose(frc::Pose3d(prevEstimatedRobotPose));
        
        //update estimate and get the result
        std::pair<frc::Pose3d, units::time::second_t> poseResult = poseEstimator->Update();

        //do we have a valid pose
        //if (poseResult) {
                //update the odometry
                driveSubsystem->visionMeasurements(poseResult.first.ToPose2d(), poseResult.second);
        //}
}

std::shared_ptr<frc::AprilTagFieldLayout> TagVision::getTagLayout() {
        return tagLayout;
}

/*photonlib::EstimatedRobotPose TagVision::getEstimatedGlobalPose(frc::Pose2d prevEstimatedRobotPose) {
        poseEstimator.setReferencePose(prevEstimatedRobotPose);
        return poseEstimator.update();
}*/
