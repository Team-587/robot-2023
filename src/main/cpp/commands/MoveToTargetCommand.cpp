// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/MoveToTargetCommand.h"

MoveToTargetCommand::MoveToTargetCommand(
  DriveSubsystem* pDriveSubsystem, 
  photonlib::PhotonCamera *pCamera, 
  PoseEstimatorSubsystem *pPoseEstimator, 
  std::string position) 
  : m_pDriveSubsystem( pDriveSubsystem ), 
    m_pCamera(pCamera),
    m_pPoseEstimator(pPoseEstimator), 
    position( position ) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({ m_pDriveSubsystem, m_pPoseEstimator });
}

MoveToTargetCommand::~MoveToTargetCommand() {
  if(alignCommand) delete alignCommand;
}

// Called when the command is initially scheduled.
void MoveToTargetCommand::Initialize() {

  //set the pipeline
  if(m_pCamera->GetPipelineIndex() != VisionPipelineIndex::APRILTAG) {
    m_pCamera->SetPipelineIndex(VisionPipelineIndex::APRILTAG);
  }
  //clean up align command
  if(alignCommand) delete alignCommand;
  alignCommand = NULL;

  //Gets the latest apriltag result from the photonlib pipeline
  photonlib::PhotonPipelineResult result = m_pCamera->GetLatestResult();

  //Runs if the camera detects an AprilTag
  if(result.HasTargets()) {

    //Sets the variable tagID equal to the fiducial ID number of the apriltag
    photonlib::PhotonTrackedTarget target = result.GetBestTarget();

    //Sets the variable targetPose equal to the position of the detected tag
    std::optional<frc::Pose3d> optionalPose = m_pPoseEstimator->GetLayout()->GetTagPose(target.GetFiducialId());
    if(optionalPose.has_value()) {
      frc::Pose3d targetPose = optionalPose.value();

      //Sets the variable visionPose equal to the transformed position of targetPose
      //transform the Pose3d
      frc::Pose2d visionPose = targetPose.ToPose2d().TransformBy( {positionMap[position].Translation(), positionMap[position].Rotation() } );

      //Sets the variable robotPose equal to the position on the field detected by the navX
      frc::Pose2d robotPose = m_pPoseEstimator->GetCurrentPose();
      
      //Creates a vector called pathPoints and sets it equal to the robot and vision position 
      std::vector<pathplanner::PathPoint> pathPoints = {
        { robotPose.Translation(), robotPose.Rotation(), robotPose.Rotation() },
        { visionPose.Translation(), visionPose.Rotation(), visionPose.Rotation() }
      };

      //Creates a new path called trajectory from the position of the robot to the detected apriltag
      pathplanner::PathPlannerTrajectory trajectory = pathplanner::PathPlanner::generatePath( constraints, pathPoints );

      //create the align command
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
      alignCommand->Initialize();
    }
  }
  if (alignCommand == NULL){
      End( true );
  }
}

// Called repeatedly when this Command is scheduled to run
void MoveToTargetCommand::Execute() { alignCommand->Execute(); }

// Called once the command ends or is interrupted.
void MoveToTargetCommand::End(bool interrupted) { alignCommand->End( interrupted ); }

// Returns true when the command should end.
bool MoveToTargetCommand::IsFinished() { return alignCommand->IsFinished(); }
