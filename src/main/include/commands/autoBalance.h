// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/DriveSubsystem.h"
#include <frc/controller/PIDController.h>

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class autoBalance
    : public frc2::CommandHelper<frc2::CommandBase, autoBalance> {
 public:
  autoBalance(DriveSubsystem* subsystem);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  DriveSubsystem *p_driveSubsystem;
  frc2::PIDController pitchPID{.017, 0.0, 0.0};
  frc2::PIDController headerPID{.02, 0.0, 0.0};

};
