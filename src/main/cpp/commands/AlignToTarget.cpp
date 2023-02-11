// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AlignToTarget.h"

#include <pathplanner/lib/commands/PPSwerveControllerCommand.h>
#include <pathplanner/lib/PathPlannerTrajectory.h>
#include <pathplanner/lib/PathPlanner.h>
#include <frc/smartdashboard/SmartDashboard.h>

AlignToTarget::AlignToTarget( DriveSubsystem* driveSubsystem, TagVision* tagVision, std::string position)
    : driveSubsystem( driveSubsystem ), tagVision( tagVision ), position( position ) {
    // Use addRequirements() here to declare subsystem dependencies.

    AddRequirements( { driveSubsystem, tagVision } );

    
}

void AlignToTarget::calculateAlignment() {
    //Gets the latest apriltag result from the photonlib pipeline
    photonlib::PhotonPipelineResult result = tagVision->camera.GetLatestResult();

    //Runs if the camera detects an AprilTag
    if(result.HasTargets()) {
        //Sets the variable tagID equal to the fiducial ID number of the apriltag
        int tagID = result.GetBestTarget().GetFiducialId();

        //Sets the variable targetPose equal to the position of the detected tag
        frc::Pose2d targetPose = tagVision->getTagLayout().GetTagPose( tagID ).value().ToPose2d();

        //Sets the variable visionPose equal to the transformed position of targetPose
        frc::Pose2d visionPose = targetPose.TransformBy( {positionMap[position].Translation(), positionMap[position].Rotation() } );

        //Sets the variable robotPose equal to the position on the field detected by the navX
        frc::Pose2d robotPose = driveSubsystem->GetPose();

        // frc::SmartDashboard::PutNumber( "Objective X", visionPose.X().value() );
        // frc::SmartDashboard::PutNumber( "Objective Y", visionPose.Y().value() );

        //Creates a vector called pathPoints and sets it equal to the robot and vision position 
        std::vector<pathplanner::PathPoint> pathPoints = {
            { robotPose.Translation(), robotPose.Rotation(), robotPose.Rotation() },
            { visionPose.Translation(), visionPose.Rotation(), visionPose.Rotation() }
        };

        //Creates a new path called trajectory from the position of the robot to the detected apriltag
        trajectory = pathplanner::PathPlanner::generatePath( constraints, pathPoints );
    } else if (!result.HasTargets()){
        End( true );
    }
}

// Called when the command is initially scheduled.
void AlignToTarget::Initialize() {
    calculateAlignment();

    alignCommand = new pathplanner::PPSwerveControllerCommand(
        trajectory,
        [this]() { return driveSubsystem->GetPose(); },
        driveSubsystem->getKinematics(),
        { 0,0,0 },
        { 0,0,0 },
        { 0,0,0 },
        [this]( auto speeds ) { driveSubsystem->SetModuleStates( speeds ); },
        { driveSubsystem },
        false
    );

    alignCommand->Initialize();
}

// Called repeatedly when this Command is scheduled to run
void AlignToTarget::Execute() {
    alignCommand->Execute();
}

// Called once the command ends or is interrupted.
void AlignToTarget::End(bool interrupted) {
    alignCommand->End( interrupted );
}

// Returns true when the command should end.
bool AlignToTarget::IsFinished() {
  return alignCommand->IsFinished();
}
