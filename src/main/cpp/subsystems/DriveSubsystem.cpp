// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DriveSubsystem.h"

#include <frc/geometry/Rotation2d.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>
#include <frc/smartdashboard/SmartDashboard.h>


#include "Constants.h"

using namespace DriveConstants;

DriveSubsystem::DriveSubsystem()
    : m_frontLeft{kFrontLeftDriveMotorPort,
                  kFrontLeftTurningMotorPort,
                  kFrontLeftAbsoluteEncoderPort,
                  kFrontLeftDriveEncoderReversed,
                  kFrontLeftTurningEncoderReversed,
                  "fl_"},

      m_rearLeft{
          kRearLeftDriveMotorPort,       kRearLeftTurningMotorPort,
          kRearLeftAbsoluteEncoderPort,
          kRearLeftDriveEncoderReversed, kRearLeftTurningEncoderReversed, "rl_"},

      m_frontRight{
          kFrontRightDriveMotorPort,       kFrontRightTurningMotorPort,
          kFrontRightAbsoluteEncoderPort,
          kFrontRightDriveEncoderReversed, kFrontRightTurningEncoderReversed, "fr_"},

      m_rearRight{
          kRearRightDriveMotorPort,       kRearRightTurningMotorPort,
          kRearRightAbsoluteEncoderPort,
          kRearRightDriveEncoderReversed, kRearRightTurningEncoderReversed, "rr_"},

      m_odometry{kDriveKinematics, 
                m_NavX.GetRotation2d(), 
                 {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),m_rearLeft.GetPosition(), m_rearRight.GetPosition()},
                 frc::Pose2d()} {}

void DriveSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
  m_odometry.Update(m_NavX.GetRotation2d(), 
                    {m_frontLeft.GetPosition(),m_rearLeft.GetPosition(), m_frontRight.GetPosition(),
                      m_rearRight.GetPosition()});
frc::SmartDashboard::PutNumber("Speed", m_fullSpeed); 
frc::SmartDashboard::PutNumber("Heading", (double)GetHeading()); 
}

void DriveSubsystem::Drive(units::meters_per_second_t xSpeed,
                           units::meters_per_second_t ySpeed,
                           units::radians_per_second_t rot,
                           bool fieldRelative) {
  if ((double)xSpeed < 0.1 && (double)xSpeed > -0.1){
    xSpeed = (units::meters_per_second_t)0.0;
  }
  if ((double)ySpeed < 0.1 && (double)ySpeed > -0.1){
    ySpeed = (units::meters_per_second_t)0.0;
  }
  if ((double)rot < 0.1 && (double)rot > -0.1){
    rot = (units::radians_per_second_t)0.0;
  }

  xSpeed = xSpeed * m_fullSpeed;
  ySpeed = ySpeed * m_fullSpeed;
  rot = rot * m_fullSpeed;

  frc::SmartDashboard::PutNumber("xSpeed", (double)xSpeed);  
  frc::SmartDashboard::PutNumber("ySpeed", (double)ySpeed);
  frc::SmartDashboard::PutNumber("Rotation", (double)rot);   
                       
  auto states = kDriveKinematics.ToSwerveModuleStates(
      fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                          xSpeed, ySpeed, rot, m_NavX.GetRotation2d())
                    : frc::ChassisSpeeds{xSpeed, ySpeed, rot});

  kDriveKinematics.DesaturateWheelSpeeds(&states, AutoConstants::kMaxSpeed);

  auto [fl, fr, bl, br] = states;
  
  m_frontLeft.SetDesiredState(fl);
  m_frontRight.SetDesiredState(fr);
  m_rearLeft.SetDesiredState(bl);
  m_rearRight.SetDesiredState(br);
}

void DriveSubsystem::SetModuleStates(
    wpi::array<frc::SwerveModuleState, 4> desiredStates) {
  kDriveKinematics.DesaturateWheelSpeeds(&desiredStates,
                                         AutoConstants::kMaxSpeed);
  m_frontLeft.SetDesiredState(desiredStates[0]);
  m_frontRight.SetDesiredState(desiredStates[1]);
  m_rearLeft.SetDesiredState(desiredStates[2]);
  m_rearRight.SetDesiredState(desiredStates[3]);
}

void DriveSubsystem::ResetEncoders() {
  m_frontLeft.ResetEncoders();
  m_rearLeft.ResetEncoders();
  m_frontRight.ResetEncoders();
  m_rearRight.ResetEncoders();
}

units::degree_t DriveSubsystem::GetHeading() const {
  return m_NavX.GetRotation2d().Degrees();
}

void DriveSubsystem::ZeroHeading() {
  m_NavX.Reset();
}

double DriveSubsystem::GetTurnRate() {
  return -m_NavX.GetRate();
}

frc::Pose2d DriveSubsystem::GetPose() {
  return m_odometry.GetPose();
}

void DriveSubsystem::limitSpeed() {
  m_fullSpeed = 0.5;
}

void DriveSubsystem::fullSpeed() {
  m_fullSpeed = 1.0;
}
void DriveSubsystem::ResetOdometry(frc::Pose2d pose) {
    m_odometry.ResetPosition(
      GetHeading(),
      {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
       m_rearLeft.GetPosition(), m_rearRight.GetPosition()},
      pose);
}
