// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <pathplanner/lib/commands/PPSwerveControllerCommand.h>
#include <pathplanner/lib/PathPlannerTrajectory.h>
#include <pathplanner/lib/PathPlanner.h>
#include <photonlib/PhotonCamera.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "subsystems/DriveSubsystem.h"
#include "subsystems/PoseEstimatorSubsystem.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class CenterCommand
    : public frc2::CommandHelper<frc2::CommandBase, CenterCommand> {
 public:
  CenterCommand(DriveSubsystem* pDriveSubsystem,
  photonlib::PhotonCamera *pCamera, PoseEstimatorSubsystem *pPoseEstimator, int pipelineIndex);
  

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  uint64_t GetCurrentTime();

  bool IsFinished() override;

  private:
  DriveSubsystem *m_pDriveSubsystem;
    photonlib::PhotonCamera *m_pCamera;
    PoseEstimatorSubsystem *m_pPoseEstimator;
    int m_pipelineIndex;
    double ySpeed;
    double xSpeed;
    double omegaSpeed;
    double lastX, lastY, lastOmega;
    uint64_t  lastTarget;
    units::degree_t targetYaw;
};
