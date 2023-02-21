// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <photonlib/PhotonCamera.h>

#include "Constants.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/PoseEstimatorSubsystem.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class AprilTagAlignCommand
    : public frc2::CommandHelper<frc2::CommandBase, AprilTagAlignCommand> {

  public:

    AprilTagAlignCommand(photonlib::PhotonCamera *pCamera, DriveSubsystem *pDriveSubsystem, PoseEstimatorSubsystem *pPoseEstimator);

    void Initialize() override;

    void Execute() override;

    void End(bool interrupted) override;

    bool IsFinished() override;

    uint64_t GetCurrentTime();

  private:
    
    //camera configured for april tags
    photonlib::PhotonCamera *m_pCamera;

    //drive subsystem
    DriveSubsystem *m_pDriveSubsystem;

    //pose estimator subsystem
    PoseEstimatorSubsystem *m_pPoseEstimator;

    frc2::PIDController pidControllerX { .5, 0.0, 0.0 };
    frc2::PIDController pidControllerY { .5, 0.0, 0.0 };
    frc2::PIDController pidControllerOmega { 1.5, 0.0, 0.0 };

    double lastX, lastY, lastOmega;
    uint64_t  lastTarget;

};
