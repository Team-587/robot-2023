// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/autoBalance.h"
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>

autoBalance::autoBalance(DriveSubsystem* subsystem):p_driveSubsystem{subsystem}{
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void autoBalance::Initialize() {
  frc::SmartDashboard::PutData("BalPid", &pitchPID);
}

// Called repeatedly when this Command is scheduled to run
void autoBalance::Execute() {
  //double tempP = frc::SmartDashboard::GetNumber("balP", 1);
  //pitchPID.SetP(tempP);
  double currentPitch = p_driveSubsystem->getPitch();
  double initialPitch = p_driveSubsystem->getInitialPitch();
  auto driveOutput = pitchPID.Calculate(currentPitch, initialPitch);

  //std::cout << "Current Pitch: " << currentPitch;
  //std::cout << " Initial Pitch: " << initialPitch;
  //std::cout << " Drive Output: " << driveOutput << "\n";
  
  if (fabs(currentPitch - initialPitch) > 15.0) {
    std::cout << "Waiting";
    p_driveSubsystem->Drive((units::velocity::meters_per_second_t)0, (units::velocity::meters_per_second_t)0, (units::angular_velocity::radians_per_second_t)0, true);
  } else {
     std::cout << "Current Pitch: " << currentPitch;
    std::cout << " Initial Pitch: " << initialPitch;
    std::cout << " Drive Output: " << driveOutput << "\n"; 
    p_driveSubsystem->Drive((units::velocity::meters_per_second_t)driveOutput, (units::velocity::meters_per_second_t)0, (units::angular_velocity::radians_per_second_t)0, true);
 }
}

// Called once the command ends or is interrupted.
void autoBalance::End(bool interrupted) {
  p_driveSubsystem->Drive((units::velocity::meters_per_second_t)0, (units::velocity::meters_per_second_t)0, (units::angular_velocity::radians_per_second_t)0, true);
}

// Returns true when the command should end.
bool autoBalance::IsFinished() {
  return false;
}
