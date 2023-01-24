// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <utility>

#include <frc/controller/PIDController.h>
#include <frc/geometry/Translation2d.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/smartdashboard/Smartdashboard.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/PrintCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <frc2/command/button/JoystickButton.h>
#include <units/angle.h>
#include <units/velocity.h>

#include "Constants.h"
#include "subsystems/DriveSubsystem.h"

using namespace DriveConstants;
using namespace pathplanner;

std::vector<PathPlannerTrajectory> RobotContainer::autoPath1 = PathPlanner::loadPathGroup("auto1", {PathConstraints(3_mps, 3_mps_sq)});
std::vector<PathPlannerTrajectory> RobotContainer::autoPath2 = PathPlanner::loadPathGroup("auto2", {PathConstraints(3_mps, 3_mps_sq)});

RobotContainer::RobotContainer(): 
  // Initialize all of your commands and subsystems here
    autoBuilder(
    [this]() { return m_drive.GetPose(); }, // Function to supply current robot pose
    [this](auto initPose) { m_drive.ResetOdometry(initPose); }, // Function used to reset odometry at the beginning of auto
    m_drive.kDriveKinematics,
    PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
      PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
      [this](auto speeds) { m_drive.SetModuleStates(speeds); }, // Output function that accepts field relative ChassisSpeeds
      eventMap, // Our event map
      {&m_drive }, // Drive requirements, usually just a single drive subsystem
      true // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
   ),
   autoNum1(autoBuilder.fullAuto(autoPath1)), 
   autoNum2(autoBuilder.fullAuto(autoPath2))
  {
    eventMap.emplace("marker1", std::make_shared<frc2::PrintCommand>("Passed Marker 1"));
    m_chooser.SetDefaultOption("Slot 1", autoNum1.get());
    m_chooser.AddOption("Slot 2", autoNum2.get());
   frc::SmartDashboard::PutData(&m_chooser);
  // Configure the button bindings
  ConfigureButtonBindings();

  // Set up default drive command
  // The left stick controls translation of the robot.
  // Turning is controlled by the X axis of the right stick.
  m_drive.SetDefaultCommand(frc2::RunCommand(
      [this] {
        m_drive.Drive(
            -units::meters_per_second_t(m_driverController.GetLeftY()),
            -units::meters_per_second_t(m_driverController.GetLeftX()),
            -units::radians_per_second_t(m_driverController.GetRightX()), true);
      },
      {&m_drive}));
}

void RobotContainer::ConfigureButtonBindings() {
    frc2::JoystickButton startButton{&m_driverController, frc::XboxController::Button::kStart};
    startButton.OnTrue(&m_ZeroHeading);
    frc2::JoystickButton LeftBumper{&m_driverController, frc::XboxController::Button::kLeftBumper};
    LeftBumper.OnTrue(&m_limitSpeed).OnFalse(&m_fullSpeed);
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // Set up config for trajectory
  frc::TrajectoryConfig config(AutoConstants::kMaxSpeed,
                               AutoConstants::kMaxAcceleration);
  // Add kinematics to ensure max speed is actually obeyed
  config.SetKinematics(m_drive.kDriveKinematics);

  // An example trajectory to follow.  All units in meters.
  auto exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
      // Start at the origin facing the +X direction
      frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
      // Pass through these two interior waypoints, making an 's' curve path
      {frc::Translation2d(1_m, 1_m), frc::Translation2d(2_m, -1_m)},
      // End 3 meters straight ahead of where we started, facing forward
      frc::Pose2d(3_m, 0_m, frc::Rotation2d(0_deg)),
      // Pass the config
      config);

  frc::ProfiledPIDController<units::radians> thetaController{
      AutoConstants::kPThetaController, 0, 0,
      AutoConstants::kThetaControllerConstraints};

  thetaController.EnableContinuousInput(units::radian_t(-std::numbers::pi),
                                        units::radian_t(std::numbers::pi));

  frc2::SwerveControllerCommand<4> swerveControllerCommand(
      exampleTrajectory, [this]() { return m_drive.GetPose(); },

      m_drive.kDriveKinematics,

      frc2::PIDController(AutoConstants::kPXController, 0, 0),
      frc2::PIDController(AutoConstants::kPYController, 0, 0), thetaController,

      [this](auto moduleStates) { m_drive.SetModuleStates(moduleStates); },

      {&m_drive});

  // Reset odometry to the starting pose of the trajectory.
  m_drive.ResetOdometry(exampleTrajectory.InitialPose());

  // no auto
  return new frc2::SequentialCommandGroup(
      std::move(swerveControllerCommand),
      frc2::InstantCommand(
          [this]() {
            m_drive.Drive(units::meters_per_second_t(0),
                          units::meters_per_second_t(0),
                          units::radians_per_second_t(0), false);
          },
          {}));
}
