// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AlignToTarget.h"

#include <pathplanner/lib/commands/PPSwerveControllerCommand.h>
#include <pathplanner/lib/PathPlannerTrajectory.h>
#include <pathplanner/lib/PathPlanner.h>
#include <photonlib/PhotonUtils.h>
#include <frc/smartdashboard/SmartDashboard.h>

AlignToTarget::AlignToTarget( DriveSubsystem *pDriveSubsystem, photonlib::PhotonCamera *pCamera, PoseEstimatorSubsystem *pPoseEstimator, std::string position)
    :   m_pDriveSubsystem( pDriveSubsystem ), 
        m_pCamera(pCamera),
        m_pPoseEstimator(pPoseEstimator), 
        position( position ) {
    // Use addRequirements() here to declare subsystem dependencies.

    AddRequirements( { m_pDriveSubsystem, m_pPoseEstimator } );

    
}

void AlignToTarget::calculateAlignment() {

    //Gets the latest apriltag result from the photonlib pipeline
    photonlib::PhotonPipelineResult result = m_pCamera->GetLatestResult();

    //Runs if the camera detects an AprilTag
    if(result.HasTargets()) {

        //Sets the variable tagID equal to the fiducial ID number of the apriltag
        photonlib::PhotonTrackedTarget target = result.GetBestTarget();

        //Sets the variable targetPose equal to the position of the detected tag
        frc::Pose3d targetPose = m_pPoseEstimator->GetLayout()->GetTagPose( target.GetFiducialId() ).value();

        //Sets the variable visionPose equal to the transformed position of targetPose
        //transform the Pose3d
        frc::Pose2d visionPose = targetPose.ToPose2d().TransformBy( {positionMap[position].Translation(), positionMap[position].Rotation() } );

        //Sets the variable robotPose equal to the position on the field detected by the navX
        //think we need this:
        frc::Pose2d robotPose = m_pPoseEstimator->GetCurrentPose();
        //frc::Pose3d robotPose = EstimateFieldToRobotAprilTag(target.GetBestCameraToTarget(), targetPose, target.GetBestCameraToTarget());
        //frc::Pose3d robotPose = frc::EstimateFieldToRobot(target.GetBestCameraToTarget(), visionPose, tagVision->GetCamerToRobot1());
        //const frc::Transform2d& cameraToTarget, const frc::Pose2d& fieldToTarget,
        //const frc::Transform2d& cameraToRobot)
        //frc::Pose2d robotPose = driveSubsystem->GetPose();

        // frc::SmartDashboard::PutNumber( "Objective X", visionPose.X().value() );
        // frc::SmartDashboard::PutNumber( "Objective Y", visionPose.Y().value() );

        //Creates a vector called pathPoints and sets it equal to the robot and vision position 
        std::vector<pathplanner::PathPoint> pathPoints = {
            //{ robotPose.ToPose2d().Translation(), robotPose.ToPose2d().Rotation(), robotPose.ToPose2d().Rotation() },
            { robotPose.Translation(), robotPose.Rotation(), robotPose.Rotation() },
            { visionPose.Translation(), visionPose.Rotation(), visionPose.Rotation() }
        };

        //Creates a new path called trajectory from the position of the robot to the detected apriltag
        trajectory = pathplanner::PathPlanner::generatePath( constraints, pathPoints );

    } else if (!result.HasTargets()){
        End( true );
    }
}

//frc::Pose3d AlignToTarget::EstimateFieldToRobotAprilTag(frc::Transform3d cameraToTarget, frc::Pose3d fieldRelativeTagPose, frc::Transform3d cameraToRobot) {
//    return fieldRelativeTagPose + cameraToTarget.Inverse() + cameraToRobot;
//}

// Called when the command is initially scheduled.
void AlignToTarget::Initialize() {
    runCount = 0;
    Start();
}

void AlignToTarget::Start() {
    
    if(alignCommand) delete alignCommand;
    
    calculateAlignment();

    alignCommand = new pathplanner::PPSwerveControllerCommand(
        trajectory,
        [this]() { return m_pPoseEstimator->GetCurrentPose(); },
        m_pDriveSubsystem->getKinematics(),
        { 0,0,0 },
        { 0,0,0 },
        { 0,0,0 },
        [this]( auto speeds ) { m_pDriveSubsystem->SetModuleStates( speeds ); },
        { m_pDriveSubsystem },
        false
    );

    runCount++;

    alignCommand->Initialize();
}

// Called repeatedly when this Command is scheduled to run
void AlignToTarget::Execute() {
    alignCommand->Execute();
}

// Called once the command ends or is interrupted.
void AlignToTarget::End(bool interrupted) {
    alignCommand->End( interrupted );
    if(alignCommand) delete alignCommand;
    alignCommand = NULL;
}

// Returns true when the command should end.
bool AlignToTarget::IsFinished() {
    if(alignCommand->IsFinished() && runCount > MAX_RUNS) {
        return true;
    } else if(alignCommand->IsFinished()) {
        Start();
        return false;
    } else {
        return false;
    }
}
