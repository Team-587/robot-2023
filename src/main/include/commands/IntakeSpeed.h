// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/Intake.h"
#include <frc/Timer.h>

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class IntakeSpeed
    : public frc2::CommandHelper<frc2::CommandBase, IntakeSpeed> {
 public:
  IntakeSpeed(Intake* subsystem, double desiredSpeed);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  Intake *p_intakeSubsystem;
  double m_desiredSpeed;
  frc::Timer m_timer;
};
