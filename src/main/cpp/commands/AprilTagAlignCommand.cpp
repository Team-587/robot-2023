// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AprilTagAlignCommand.h"
#include <iostream>
#include <photonlib/PhotonUtils.h>
#include <chrono>

AprilTagAlignCommand::AprilTagAlignCommand(photonlib::PhotonCamera *pCamera, DriveSubsystem *pDriveSubsystem, PoseEstimatorSubsystem *pPoseEstimator) 
  :   m_pCamera(pCamera),
      m_pDriveSubsystem(pDriveSubsystem),
      m_pPoseEstimator(pPoseEstimator) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({ m_pDriveSubsystem, m_pPoseEstimator });

  pidControllerX.Reset();
  pidControllerY.Reset();
  pidControllerOmega.Reset();

  pidControllerX.SetSetpoint((double)0.6); // Move forward/backwork to keep 36 inches from the target
  pidControllerX.SetTolerance((double)0.01);

  pidControllerY.SetSetpoint(0); // Move side to side to keep target centered
  pidControllerY.SetTolerance((double).01);

  pidControllerOmega.SetSetpoint((double)units::radian_t(180_deg)); // Rotate the keep perpendicular with the target
  pidControllerOmega.SetTolerance((double)units::radian_t(10_deg));

}

uint64_t AprilTagAlignCommand::GetCurrentTime() {
  return duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}

// Called when the command is initially scheduled.
void AprilTagAlignCommand::Initialize() {

  std::cout << "I'm initializing" << "\n";

  lastX = 0;
  lastY = 0;
  lastOmega = 0;
  lastTarget = 0;

  
}

// Called repeatedly when this Command is scheduled to run
void AprilTagAlignCommand::Execute() {

  //std::cout << "I'm executing" << "\n";

  photonlib::PhotonPipelineResult result = m_pCamera->GetLatestResult();

  if (result.HasTargets()) {

    //std::cout << "target detected" << "\n";

    frc::Transform3d cameraToTarget = result.GetBestTarget().GetBestCameraToTarget();

    // X - distance from camera in meters
    // Y - right and left of camera center (in meters?)
    // Z - above and below camera center (in meters?)
    // rotation X - pitch - 0-degrees is flat on floor - rotation is positive as tilted toward camera
    //                    - visible targets in range [0, 180]
    // rotation Y - roll - 0-degrees is straight upward (or straight down) - clockwise rotation is positive
    //                  - seems to give same results if the target is upside down (maybe need to research this one)
    //                  - visible targets are in range [-90, 90]
    // rotation Z - yaw - 0-degrees is perpendicular to the camera, rotated with the right side away from camera (not visible)
    //                  - -90-degrees is straight on with the camera
    //                  - from the camera's perspective, rotation the left side of the target closer is positive
    //                  - visible targets are in range [-180, 0]

    frc::SmartDashboard::PutData("PID X", &pidControllerX);
    frc::SmartDashboard::PutData("PID Y", &pidControllerY);
    frc::SmartDashboard::PutData("PID Omega", &pidControllerOmega);


    cameraToTarget.Rotation().Angle();
    frc::SmartDashboard::PutNumber("Target X", (double)units::meter_t(cameraToTarget.X()));
    frc::SmartDashboard::PutNumber("Target Y", (double)units::meter_t(cameraToTarget.Y()));
    frc::SmartDashboard::PutNumber("Target Z", (double)units::meter_t(cameraToTarget.Z()));
    frc::SmartDashboard::PutNumber("Target Rotation X", (double)units::radian_t(cameraToTarget.Rotation().X()));
    frc::SmartDashboard::PutNumber("Target Rotation Y", (double)units::radian_t(cameraToTarget.Rotation().Y()));
    frc::SmartDashboard::PutNumber("Target Rotation Z", (double)units::radian_t(cameraToTarget.Rotation().Z()));

    
    units::meter_t range = photonlib::PhotonUtils::CalculateDistanceToTarget(
          Camerapos::cam_height_meters, 
          FieldElementsMeasurement::april_tag_height, 
          Camerapos::cam_angle_degrees,
          units::degree_t{result.GetBestTarget().GetPitch()});

    // Handle distance to target
    //units::length::meter_t distanceFromTarget = cameraToTarget.X();
    double xSpeed = -pidControllerX.Calculate((double)range);
    frc::SmartDashboard::PutNumber("xAlignSpeed", xSpeed);
    //frc::SmartDashboard::PutBoolean("X SetPoint", pidControllerX.AtSetpoint());
    frc::SmartDashboard::PutNumber("Range", (double)range);
    
    //xSpeed = 0.1; 

    //if ((double)range < 0.6) {
    if (pidControllerX.AtSetpoint()) {
      xSpeed = 0;
    } 
      


    // Handle alignment side-to-side
    units::length::meter_t targetY = cameraToTarget.Y();
    double ySpeed = -pidControllerY.Calculate((double)targetY);
    //ySpeed = (double)targetY > 0 ? 0.3 : -0.3;
    
    frc::SmartDashboard::PutNumber("yAlignSpeed", ySpeed);
    
    if (pidControllerY.AtSetpoint()) {
      ySpeed = 0;
    }

//    // Handle rotation using target Yaw/Z rotation
    units::radian_t targetYaw = cameraToTarget.Rotation().Z();
    double omegaSpeed = -pidControllerOmega.Calculate((double)targetYaw);
    //omegaSpeed = (double)targetYaw > 0 ? -0.1 : 0.1;

    frc::SmartDashboard::PutNumber("omegaAlignSpeed", omegaSpeed);
     if (pidControllerOmega.AtSetpoint()) {
      omegaSpeed = 0;
    }
    
    //std::cout << "before drive" << "\n";

    lastX = xSpeed;
    lastY = ySpeed;
    lastOmega = omegaSpeed;
    lastTarget = GetCurrentTime();

    m_pDriveSubsystem->Drive(
      -units::meters_per_second_t(xSpeed), 
      -units::meters_per_second_t(ySpeed), 
      -units::radians_per_second_t(omegaSpeed),
      true);
  } else if(GetCurrentTime() - lastTarget < 500) {
      m_pDriveSubsystem->Drive(
      -units::meters_per_second_t(lastX), 
      -units::meters_per_second_t(lastY), 
      -units::radians_per_second_t(lastOmega),
      true);
  } else {
     m_pDriveSubsystem->Stop();
  }
}

// Called once the command ends or is interrupted.
void AprilTagAlignCommand::End(bool interrupted) {
  m_pDriveSubsystem->Stop();
}

// Returns true when the command should end.
bool AprilTagAlignCommand::IsFinished() {
  return CommandBase::IsFinished();
}
