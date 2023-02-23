// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/CenterCommand.h"
#include <math.h>

CenterCommand::CenterCommand(DriveSubsystem* pDriveSubsystem,
  photonlib::PhotonCamera *pCamera, PoseEstimatorSubsystem *pPoseEstimator, int pipelineIndex) 
  // Use addRequirements() here to declare subsystem dependencies. 
  : m_pDriveSubsystem( pDriveSubsystem ), 
    m_pCamera(pCamera),
    m_pPoseEstimator(pPoseEstimator),
    m_pipelineIndex(pipelineIndex) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({ m_pDriveSubsystem, m_pPoseEstimator });
}


// Called when the command is initially scheduled.
void CenterCommand::Initialize() {
  if(m_pCamera->GetPipelineIndex() != m_pipelineIndex) {
    m_pCamera->SetPipelineIndex(m_pipelineIndex);

  }

  
}

// Called repeatedly when this Command is scheduled to run
void CenterCommand::Execute() {
  //Gets the latest apriltag result from the photonlib pipeline
  photonlib::PhotonPipelineResult result = m_pCamera->GetLatestResult();

  //Runs if the camera detects an AprilTag
  if(result.HasTargets()) {
    //Sets the variable tagID equal to the fiducial ID number of the apriltag
    photonlib::PhotonTrackedTarget target = result.GetBestTarget();
    //double yaw = target.GetYaw();
    frc::Transform3d cameraToTarget = result.GetBestTarget().GetBestCameraToTarget();
    units::length::meter_t targetY = cameraToTarget.Y();
    ySpeed = (double)targetY > 0 ? 0.3 : -0.3;
    
    if(abs((double)targetY) < 3.0) {
      ySpeed = 0.0;
    }
    
    units::radian_t targetYaw = cameraToTarget.Rotation().Z();
    omegaSpeed = (double)targetYaw > 0 ? -0.1 : 0.1;

    if(abs((double)targetYaw) < 3.0) {
      omegaSpeed = 0.0;
    }
    m_pDriveSubsystem->Drive(
      -units::meters_per_second_t(0),
      -units::meters_per_second_t(ySpeed),
      units::radians_per_second_t(omegaSpeed),
      true);
  } else {
    End(true);
  }

  
}

// Called once the command ends or is interrupted.
void CenterCommand::End(bool interrupted) { m_pDriveSubsystem->Stop(); }

// Returns true when the command should end.
bool CenterCommand::IsFinished() {
  return ySpeed == 0.0 && omegaSpeed == 0.0;

}
