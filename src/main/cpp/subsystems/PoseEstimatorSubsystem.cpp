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
  //setup suffleboard
  frc::ShuffleboardTab &tab = frc::Shuffleboard::GetTab("Drivetrain");
  //tab.AddString("PES (X, Y)", [this]() { return GetFomattedPose(); });
  //tab.AddNumber("PES Degrees", [this]() { return (double)GetCurrentPose().Rotation().Degrees(); });
  tab.Add("PES::Field2d", field2d);
  
  m_WaitCommand.Initialize();
  
  //update odometry
  m_pPoseEstimator->UpdateWithTime(frc::Timer::GetFPGATimestamp(), m_pDriveSubsystem->GetGyroscopeRotation(), m_pDriveSubsystem->GetOdometryPos());
  field2d.SetRobotPose(GetCurrentPose());
}

  PoseEstimatorSubsystem::~PoseEstimatorSubsystem() {
    if(m_pPoseEstimator) delete m_pPoseEstimator;
  }

// This method will be called once per scheduler run
void PoseEstimatorSubsystem::Periodic() { }

//update the estimator
void PoseEstimatorSubsystem::Update() {
  //set the pipeline
  if(m_pCamera->GetPipelineIndex() != VisionPipelineIndex::APRILTAG) {
    m_pCamera->SetPipelineIndex(VisionPipelineIndex::APRILTAG);
  }
  
  //delay the start of the PoseEstimator
  if(!m_WaitCommand.IsFinished()) { return; }

  // Update pose estimator with visible targets
  photonlib::PhotonPipelineResult result = m_pCamera->GetLatestResult();

  //check for targets
  if (result.HasTargets()) {
    //std::cout << "Hey I'm here, I didn't crash! 1\n";
    //get the image latency
    units::time::second_t imageCaptureTime = frc::Timer::GetFPGATimestamp() - result.GetLatency();
    //std::cout << "Hey I'm here, I didn't crash! 2\n";
    //parse through all found apriltags
    for (photonlib::PhotonTrackedTarget target : result.GetTargets()) {
    //std::cout << "Hey I'm here, I didn't crash! 3\n";
      //get the ID
      int fiducialId = target.GetFiducialId();
      //std::cout << "Hey I'm here, I didn't crash! 4\n";
      //make sure ID is valid
      if (fiducialId  >= 0) {
//std::cout << "Hey I'm here, I didn't crash! 5\n";
        //get the pose of the target
        std::optional<frc::Pose3d> optionalPose = tagLayout.GetTagPose(fiducialId);
        if(optionalPose.has_value()) {
          frc::Pose2d targetPose = optionalPose.value().ToPose2d();
//std::cout << "Hey I'm here, I didn't crash! 6\n";
          //get the camera pose
          frc::Transform3d camToTarget = target.GetBestCameraToTarget();
//          std::cout << "Hey I'm here, I didn't crash! 7\n";
          frc::Transform2d transform = frc::Transform2d(
            camToTarget.Translation().ToTranslation2d(), 
            camToTarget.Rotation().ToRotation2d());// - frc::Rotation2d(90_deg));
          frc::Pose2d camPose = targetPose.TransformBy(transform.Inverse());
//          std::cout << "Hey I'm here, I didn't crash! 8\n";

          //record vision measurement
          frc::Pose2d visionMeasurement = camPose.TransformBy(CAMERA_TO_ROBOT);
          //std::cout << "Hey I'm here, I didn't crash! :" << (double)visionMeasurement.X() << "\n";
          //std::cout << "Hey I'm here, I didn't crash! :" << (double)visionMeasurement.Y() << "\n";
          //std::cout << "Hey I'm here, I didn't crash! :" << (double)visionMeasurement.Rotation().Degrees() << "\n";
          //std::cout << "Hey I'm here, I didn't crash! :" << (uint64_t)imageCaptureTime << "\n";
          //field2d.GetObject("MyRobot:" + fiducialId)->SetPose(visionMeasurement);
          if (m_pPoseEstimator) {
            //std::cout << "Hey I'm here, I didn't crash!\n";
          }
          try{
            m_pPoseEstimator->AddVisionMeasurement(visionMeasurement, imageCaptureTime);
          } catch(std::exception& e) {
            std::cerr << e.what() << "\n";
          }
          //std::cout << "Hey I'm here, I didn't crash! 9\n";
        }
      }
    }
  }
  
  //update odometry
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

