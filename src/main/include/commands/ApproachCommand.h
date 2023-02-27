// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <photonlib/PhotonCamera.h>

#include "subsystems/DriveSubsystem.h"
#include "subsystems/PoseEstimatorSubsystem.h"
/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class ApproachCommand
    : public frc2::CommandHelper<frc2::CommandBase, ApproachCommand> {
 public:
  ApproachCommand(DriveSubsystem* pDriveSubsystem, photonlib::PhotonCamera *pCamera, PoseEstimatorSubsystem * pPoseEstimator, int pipelineIndex);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  private:
  DriveSubsystem *m_pDriveSubsystem;
  photonlib::PhotonCamera *m_pCamera;
  PoseEstimatorSubsystem *m_pPoseEstimator;
};
