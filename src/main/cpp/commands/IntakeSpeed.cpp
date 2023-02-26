// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/IntakeSpeed.h"

IntakeSpeed::IntakeSpeed(Intake* subsystem, double desiredSpeed):p_intakeSubsystem{subsystem}, m_desiredSpeed{desiredSpeed} {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(p_intakeSubsystem);
}

// Called when the command is initially scheduled.
void IntakeSpeed::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void IntakeSpeed::Execute() {
  p_intakeSubsystem->checkControl(m_desiredSpeed);
}

// Called once the command ends or is interrupted.
void IntakeSpeed::End(bool interrupted) {
  p_intakeSubsystem->checkControl(0);
}

// Returns true when the command should end.
bool IntakeSpeed::IsFinished() {
  return false;
}
