// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <rev/CANSparkMax.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/controller/PIDController.h>
#include <Constants.h>
#include <frc/DigitalOutput.h>

#define ELEVATOR_VALID 

class Elevator : public frc2::SubsystemBase {
 public:
  Elevator();

 void setElevatorPosition(double position);

 void zeroElevatorHead();

  void Periodic() override;

  void ToggleColor();
 private:
  //double position = 0;
  double destination = 0;
  frc::DigitalOutput color1;
  frc::DigitalOutput color2;
  int currentColor = 0;

  #ifdef ELEVATOR_VALID
  rev::CANSparkMax m_elevatorMotor1 {DriveConstants::kElevatorMotor1, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
  rev::CANSparkMax m_elevatorMotor2 {DriveConstants::kElevatorMotor2, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
  rev::SparkMaxRelativeEncoder encoderMotor1;
  rev::SparkMaxPIDController PID_motor1;




  #endif
  frc2::PIDController elevatorPID{0.054, 0.0, 0.0};
  double P = 0.054;
  double I = 0.0;
  double D = 0.0;
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
