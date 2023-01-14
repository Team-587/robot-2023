// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SwerveModule.h"

#include <frc/geometry/Rotation2d.h>


#include "Constants.h"

SwerveModule::SwerveModule(int driveMotorChannel, int turningMotorChannel,
                           const int absoluteEncoderChannel,
                           bool driveEncoderReversed,
                           bool turningEncoderReversed)
    : m_driveMotor(driveMotorChannel, rev::CANSparkMaxLowLevel::MotorType::kBrushless),
      m_turningMotor(turningMotorChannel, rev::CANSparkMaxLowLevel::MotorType::kBrushless),
      m_absoluteEncoder(absoluteEncoderChannel),
      m_drive_encoder(m_driveMotor.GetEncoder()),
      m_reverseDriveEncoder(driveEncoderReversed),
      m_reverseTurningEncoder(turningEncoderReversed) {

  m_absoluteEncoder.SetPositionToAbsolute();

    m_driveMotor.RestoreFactoryDefaults();
    m_driveMotor.SetInverted(m_reverseDriveEncoder);
    m_driveMotor.SetSmartCurrentLimit(50);
    m_driveMotor.SetSecondaryCurrentLimit(80);
    m_driveMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_driveMotor.EnableVoltageCompensation(12);
    //m_drive_encoder = &m_driveMotor.GetEncoder();

    m_drive_encoder.SetPositionConversionFactor(0.319 / 6.12);
    m_drive_encoder.SetVelocityConversionFactor((0.319 / 6.12)/60.0); // wheel circumfrence meters / gear reduction
 
    m_turningMotor.RestoreFactoryDefaults();
    m_turningMotor.SetInverted(m_reverseTurningEncoder);
    m_turningMotor.SetSmartCurrentLimit(50);
    m_turningMotor.SetSecondaryCurrentLimit(80);

    m_turningMotor.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus0, 100);
    m_turningMotor.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus2, 100);   
    m_turningMotor.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus1, 100);
    m_driveMotor.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus0, 100);
    m_driveMotor.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus1, 20);
    m_driveMotor.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus2, 100);   
    m_absoluteEncoder.SetStatusFramePeriod(ctre::phoenix::sensors::CANCoderStatusFrame::CANCoderStatusFrame_SensorData, 20);

  // Limit the PID Controller's input range between -pi and pi and set the input
  // to be continuous.
  m_turningPIDController.EnableContinuousInput(
      -std::numbers::pi, std::numbers::pi);
}

frc::SwerveModuleState SwerveModule::GetState() {
  return {units::meters_per_second_t{m_drive_encoder.GetVelocity()},
  //Subject to change
          //frc::Rotation2d(units::radian_t(m_turningEncoder.Get()))};
          //Sketchy potentially error filled code!!!!!
          frc::Rotation2d(units::radian_t(m_absoluteEncoder.GetPosition()))};
}

frc::SwerveModulePosition SwerveModule::GetPosition() {
  //double encPos{m_absoluteEncoder.GetPosition()};
  return {units::meter_t{m_drive_encoder.GetPosition()},
        units::radian_t{m_absoluteEncoder.GetPosition()}};
}

void SwerveModule::SetDesiredState(
    const frc::SwerveModuleState& referenceState) {
  // Optimize the reference state to avoid spinning further than 90 degrees
  const auto state = frc::SwerveModuleState::Optimize(
      referenceState, units::radian_t(m_absoluteEncoder.GetPosition()));

  // Calculate the drive output from the drive PID controller.
  /*const auto driveOutput = m_drivePIDController.Calculate(
      m_driveEncoder.GetRate(), state.speed.value());*/

  // Calculate the turning motor output from the turning PID controller.
  auto turnOutput = m_turningPIDController.Calculate(
      m_absoluteEncoder.GetPosition(), state.angle.Radians().to<double>());

  // Set the motor outputs.
  //m_driveMotor.Set(driveOutput);
  m_turningMotor.Set(turnOutput);
}

void SwerveModule::SetDesiredAutoState(
    const frc::SwerveModuleState& referenceState) {
  // Optimize the reference state to avoid spinning further than 90 degrees
  double angle = m_absoluteEncoder.GetAbsolutePosition();
  angle = angle * (std::numbers::pi / 180.0) - std::numbers::pi;
  //const auto state = frc::SwerveModuleState::Optimize(
      //referenceState, units::radian_t(angle));
  const auto state = referenceState;
  // Calculate the drive output from the drive PID controller.
    const auto driveOutput = m_drivePIDController.Calculate(
    m_drive_encoder.GetVelocity(), state.speed.value());
    const auto driveFeedforward = m_driveFeedForward.Calculate(state.speed);

  // Calculate the turning motor output from the turning PID controller.
  //auto turnOutput = m_turningPIDController.Calculate(
      //units::radian_t(m_turningEncoder.Get()), state.angle.Radians());

  // frc::SmartDashboard::PutNumber(m_name + " TurnAngle", state.angle.Radians().to<double>());
  // frc::SmartDashboard::PutNumber(m_name + " CurAngle", angle);
  // frc::SmartDashboard::PutNumber(m_name + " Speed", state.speed.to<double>());
  //double temp_p = frc::SmartDashboard::GetNumber("PValue", turnP);
  //double temp_i = frc::SmartDashboard::GetNumber("IValue", turnI);
  //double temp_d = frc::SmartDashboard::GetNumber("DValue", turnD);

  //m_turningPIDController.SetPID(temp_p, temp_i, temp_d);
  double output = m_turningPIDController.Calculate(angle, state.angle.Radians().to<double>());
  if (output > 1.0) output = 1.0;
  if (output < -1.0) output = -1.0;

  // Set the motor outputs.
  //m_driveMotor.Set(referenceState.speed.to<double>());
  m_driveMotor.SetVoltage(units::volt_t{driveOutput} + driveFeedforward);
  //m_revDrivePIDController.SetReference(referenceState.speed.to<double>() * AutoConstants::kMaxSpeed.to<double>(), rev::ControlType::kVelocity);
  
  //m_driveMotor.Set(0);
  m_turningMotor.Set(output);
}

void SwerveModule::Stop()
{
   m_driveMotor.Set(0);
   m_turningMotor.Set(0);
}

void SwerveModule::ResetEncoders() {
  //m_drive_encoder.SetPosition(0);
  m_absoluteEncoder.SetPosition(0);
}
