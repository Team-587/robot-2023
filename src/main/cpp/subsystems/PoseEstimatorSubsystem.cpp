// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/PoseEstimatorSubsystem.h"

PoseEstimatorSubsystem::PoseEstimatorSubsystem(photonlib::PhotonCamera *pCamera, DriveSubsystem *pDriveSubsystem)
  :   m_pCamera(pCamera),
      m_pDriveSubsystem(pDriveSubsystem) {

  // Kalman Filter Configuration. These can be "tuned-to-taste" based on how much
  // you trust your various sensors. Smaller numbers will cause the filter to
  // "trust" the estimate from that particular component more than the others. 
  // This in turn means the particualr component will have a stronger influence
  // on the final pose estimate.
  m_pPoseEstimator = new frc::SwerveDrivePoseEstimator<4>(
    m_pDriveSubsystem->kDriveKinematics,
    m_pDriveSubsystem->GetGyroscopeRotation(),//frc::Rotation2d{},
    m_pDriveSubsystem->GetOdometryPos(),
    frc::Pose2d{},
    { 0.1, 0.1, (double)units::radian_t(5_deg) },
    { 0.1, 0.1, (double)units::radian_t(5_deg) }
  );

  //get the alliance color
  frc::DriverStation::Alliance allianceColor = frc::DriverStation::GetAlliance();

  //set the origin
  if (allianceColor == frc::DriverStation::Alliance::kBlue) {
          tagLayout.SetOrigin(frc::AprilTagFieldLayout::OriginPosition::kBlueAllianceWallRightSide);
  } else {
          tagLayout.SetOrigin(frc::AprilTagFieldLayout::OriginPosition::kRedAllianceWallRightSide);
  }

  frc::ShuffleboardTab &tab = frc::Shuffleboard::GetTab("Drivetrain");
  tab.AddString("PES (X, Y)", [this]() { return GetFomattedPose(); });
  tab.AddNumber("PES Degrees", [this]() { return (double)GetCurrentPose().Rotation().Degrees(); });
  tab.Add("PES::Field2d", field2d);
}

// This method will be called once per scheduler run
void PoseEstimatorSubsystem::Periodic() { }

//update the estimator
void PoseEstimatorSubsystem::Update() {
  //set the pipeline
  if(m_pCamera->GetPipelineIndex() != VisionPipelineIndex::APRILTAG) {
    m_pCamera->SetPipelineIndex(VisionPipelineIndex::APRILTAG);
  }
  // Update pose estimator with visible targets
  photonlib::PhotonPipelineResult result = m_pCamera->GetLatestResult();

    if (result.HasTargets()) {

      units::time::second_t imageCaptureTime = frc::Timer::GetFPGATimestamp() - result.GetLatency();

      for (photonlib::PhotonTrackedTarget target : result.GetTargets()) {

        int fiducialId = target.GetFiducialId();

        if (fiducialId  >= 0) {

          frc::Pose2d targetPose = tagLayout.GetTagPose(fiducialId).value().ToPose2d();

          frc::Transform3d camToTarget = target.GetBestCameraToTarget();

          frc::Transform2d transform = frc::Transform2d(
            camToTarget.Translation().ToTranslation2d(), 
            camToTarget.Rotation().ToRotation2d());// - frc::Rotation2d(90_deg));

          frc::Pose2d camPose = targetPose.TransformBy(transform.Inverse());

          frc::Pose2d visionMeasurement = camPose.TransformBy(CAMERA_TO_ROBOT);
          field2d.GetObject("MyRobot:" + fiducialId)->SetPose(visionMeasurement);
          //.GetObject("MyRobot" + fiducialId).SetPose(visionMeasurement);
          // SmartDashboard.putString("Vision pose", String.format("(%.2f, %.2f) %.2f",
          //   visionMeasurement.getTranslation().getX(),
          //   visionMeasurement.getTranslation().getY(),
          //   visionMeasurement.getRotation().getDegrees()));
          m_pPoseEstimator->AddVisionMeasurement(visionMeasurement, imageCaptureTime);
        }
      }
          // Update pose estimator with drivetrain sensors
    }
    m_pPoseEstimator->UpdateWithTime(frc::Timer::GetFPGATimestamp(), m_pDriveSubsystem->GetGyroscopeRotation(), m_pDriveSubsystem->GetOdometryPos());

    field2d.SetRobotPose(GetCurrentPose());
}

std::string PoseEstimatorSubsystem::GetFomattedPose() {
    frc::Pose2d pose = GetCurrentPose();
    char buff[100];
    snprintf(buff, sizeof(buff), "(%.2f, %.2f)", 
      (double)units::inch_t(pose.X()), (double)units::inch_t(pose.Y()));
    std::string buffAsStdStr = buff;
    return buffAsStdStr;
  //return "test";
    /*return String.format("(%.2f, %.2f)", 
        Units.metersToInches(pose.getX()), 
        Units.metersToInches(pose.getY()));*/
  }

  frc::Pose2d PoseEstimatorSubsystem::GetCurrentPose() {
    return m_pPoseEstimator->GetEstimatedPosition();
  }

  /**
   * Resets the current pose to the specified pose. This should ONLY be called
   * when the robot's position on the field is known, like at the beginning of
   * a match.
   * @param newPose new pose
   */
  void PoseEstimatorSubsystem::SetCurrentPose(frc::Pose2d newPose) {
    m_pDriveSubsystem->ResetGyroAngle();
    m_pPoseEstimator->ResetPosition(m_pDriveSubsystem->GetGyroscopeRotation(), m_pDriveSubsystem->GetOdometryPos(), newPose);
  }

  /**
   * Resets the position on the field to 0,0 0-degrees, with forward being downfield. This resets
   * what "forward" is for field oriented driving.
   */
  void PoseEstimatorSubsystem::ResetFieldPosition() {
    m_pDriveSubsystem->ResetGyroAngle();
    m_pPoseEstimator->ResetPosition(m_pDriveSubsystem->GetGyroscopeRotation(), m_pDriveSubsystem->GetOdometryPos(), frc::Pose2d());
  }

