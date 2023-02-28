// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ApproachCommand.h"

ApproachCommand::ApproachCommand(DriveSubsystem* pDriveSubsystem, photonlib::PhotonCamera *pCamera, PoseEstimatorSubsystem * pPoseEstimator, double speed) 
:
  // Use addRequirements() here to declare subsystem dependencies.
  m_pDriveSubsystem( pDriveSubsystem ), 
  m_pCamera(pCamera),
  m_pPoseEstimator(pPoseEstimator),
  m_speed(speed) {

    AddRequirements({ m_pDriveSubsystem, m_pPoseEstimator });

  }



// Called when the command is initially scheduled.
void ApproachCommand::Initialize() {
  
}

// Called repeatedly when this Command is scheduled to run
void ApproachCommand::Execute() {
  m_pDriveSubsystem->Drive(
    -units::meters_per_second_t(m_speed),
    -units::meters_per_second_t(0),
    units::radians_per_second_t(0),
    true);
}

// Called once the command ends or is interrupted.
void ApproachCommand::End(bool interrupted) {}

// Returns true when the command should end.
bool ApproachCommand::IsFinished() {
  return false;
}
