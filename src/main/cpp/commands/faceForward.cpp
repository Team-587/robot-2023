// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/faceForward.h"
#include <iostream>

faceForward::faceForward(DriveSubsystem* subsystem):p_driveSubsystem{subsystem} {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(subsystem);
}

// Called when the command is initially scheduled.
void faceForward::Initialize() {
  headerPID.EnableContinuousInput(-180, 180);
}

// Called repeatedly when this Command is scheduled to run
void faceForward::Execute() {
  double currentYaw = p_driveSubsystem->getCurrentYaw();
  double desiredYaw = 180.0;

  auto rotateOutput = headerPID.Calculate(currentYaw, desiredYaw);

  std::cout << " Current Yaw: " << currentYaw;
  std::cout << " Desired Yaw: " << desiredYaw;
  std::cout << " Rotate Output: " << rotateOutput << "\n";

  p_driveSubsystem->Drive((units::velocity::meters_per_second_t)0, (units::velocity::meters_per_second_t)0, (units::angular_velocity::radians_per_second_t)rotateOutput, true);

}

// Called once the command ends or is interrupted.
void faceForward::End(bool interrupted) {
  p_driveSubsystem->Drive((units::velocity::meters_per_second_t)0, (units::velocity::meters_per_second_t)0, (units::angular_velocity::radians_per_second_t)0, true);

}

// Returns true when the command should end.
bool faceForward::IsFinished() {
  return false;
}
