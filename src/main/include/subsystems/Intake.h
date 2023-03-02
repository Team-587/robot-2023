// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include <frc/DoubleSolenoid.h>
#include <Constants.h>

#define INTAKE_VALID 

class Intake : public frc2::SubsystemBase {
 public:
  Intake();
  void checkControl(double wheelSpeed);
  void autoSpeed(double speed);
  void setAuto(bool val);
  void extended(bool extend);
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  #ifdef INTAKE_VALID
  rev::CANSparkMax m_intakeMotor {DriveConstants::kIntakeMotor, rev::CANSparkMaxLowLevel::MotorType::kBrushed};
  frc::DoubleSolenoid m_intakeSolenoid {frc::PneumaticsModuleType::CTREPCM, kIntakeSolenoid, kIntakeSolenoid1};
  #endif

  double m_speed = 0.0;
  bool in_auto = false;
};
