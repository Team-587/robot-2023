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
    std::cout << "Cout statements my beloved 1\n";
    //Sets the variable tagID equal to the fiducial ID number of the apriltag
    photonlib::PhotonTrackedTarget target = result.GetBestTarget();
    //double yaw = target.GetYaw();
    frc::Transform3d cameraToTarget = result.GetBestTarget().GetBestCameraToTarget();
    std::cout << "Cout statements my beloved 2\n";
    units::length::meter_t targetY = cameraToTarget.Y();
    ySpeed = (double)targetY > 0 ? -0.2 : 0.2;
    std::cout << "Cout statements my beloved 3 " << (double)targetY << "\n";
     if(abs((double)targetY) < 0.2) {
      std::cout << "Cout statements my beloved 4\n";
      ySpeed = 0.0;
     }
    
    units::degree_t targetYaw = cameraToTarget.Rotation().Z();
    omegaSpeed = (double)targetYaw > 0 ? -0.1 : 0.1;

    std::cout << "Cout statements my beloved 5 " << (double)targetYaw << "\n";
    if(abs((double)targetYaw) < .3) {
      
      omegaSpeed = 0.0;
    }

    lastX = xSpeed;
    lastY = ySpeed;
    lastOmega = omegaSpeed;
    lastTarget = GetCurrentTime();

    m_pDriveSubsystem->Drive(
      -units::meters_per_second_t(0),
      -units::meters_per_second_t(ySpeed),
      units::radians_per_second_t(omegaSpeed),
      true);
      std::cout << "Cout statements my beloved 6\n";
  } else if(GetCurrentTime() - lastTarget < 800) {
      m_pDriveSubsystem->Drive(
      -units::meters_per_second_t(0), 
      -units::meters_per_second_t(lastY), 
      -units::radians_per_second_t(lastOmega),
      true);
  } else {
    std::cout << "Cout statements my beloved 7\n";
    End(true);
  }

  
}
uint64_t CenterCommand::GetCurrentTime() {
  return duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}

// Called once the command ends or is interrupted.
void CenterCommand::End(bool interrupted) { m_pDriveSubsystem->Stop(); }

// Returns true when the command should end.
bool CenterCommand::IsFinished() {
  std::cout << "Cout statements my beloved 8\n";
  return ySpeed == 0.0 && omegaSpeed == 0.0 && 180 - abs((double)targetYaw) < 2;
}
