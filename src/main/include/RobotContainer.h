// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/XboxController.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/Command.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/PIDCommand.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/PrintCommand.h>
#include <pathplanner/lib/PathPlanner.h>
#include <pathplanner/lib/PathConstraints.h>
#include <pathplanner/lib/PathPlannerTrajectory.h>
#include <pathplanner/lib/PathPoint.h>
#include <pathplanner/lib/GeometryUtil.h>
#include <pathplanner/lib/auto/SwerveAutoBuilder.h>
#include <pathplanner/lib/commands/FollowPathWithEvents.h>
#include <frc2/command/CommandPtr.h>

#include "Constants.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/Intake.h"
#include "commands/autoBalance.h"
#include "subsystems/Elevator.h"
#include "commands/IntakeSpeed.h"

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();

  frc2::Command* GetAutonomousCommand();

 private:
  // The driver's controller
  frc::XboxController m_driverController{OIConstants::kDriverControllerPort};
  frc::XboxController m_coDriverController{OIConstants::kCoDriverControllerPort};
  // The robot's subsystems and commands are defined here...

  // The robot's subsystems
  DriveSubsystem m_drive;
public:
  Intake m_intake;
private:
  Elevator m_elevator;

  // The chooser for the autonomous routines
  frc::SendableChooser<frc2::Command*> m_chooser;

  void ConfigureButtonBindings();
  frc2::InstantCommand m_ZeroHeading{[this] {m_drive.ZeroHeading(); }, {&m_drive}};
  frc2::InstantCommand m_halfSpeed{[this] {m_drive.limitSpeed(0.5); }, {&m_drive}};
  frc2::InstantCommand m_quarterSpeed{[this] {m_drive.limitSpeed(0.2); }, {&m_drive}};
  frc2::InstantCommand m_fullSpeed{[this] {m_drive.fullSpeed(); }, {&m_drive}};
  frc2::InstantCommand m_extendIntake{[this] {m_intake.extended(false); }, {&m_intake}};
  frc2::InstantCommand m_runIntake{[this] {m_intake.autoSpeed(0.5); }, {&m_intake}};
  frc2::InstantCommand m_runIntakeOpposite{[this] {m_intake.autoSpeed(-0.5); }, {&m_intake}};
  frc2::InstantCommand m_stopIntake{[this] {m_intake.autoSpeed(0.0); }, {&m_intake}};
  //IntakeSpeed m_runIntake{&m_intake, 0.50};
  //IntakeSpeed m_runIntakeOpposite{&m_intake, -0.50};
  //IntakeSpeed m_stopIntake{&m_intake, 0.0};
  frc2::InstantCommand m_retractIntake{[this] {m_intake.extended(true); }, {&m_intake}};

  frc2::InstantCommand m_elevatorDown{[this] {m_elevator.setElevatorPosition(kElevatorDown); }, {&m_elevator}};
  frc2::InstantCommand m_elevatorLow{[this] {m_elevator.setElevatorPosition(kElevatorLow); }, {&m_elevator}};
  frc2::InstantCommand m_elevatorMid{[this] {m_elevator.setElevatorPosition(kElevatorMid); }, {&m_elevator}};
  frc2::InstantCommand m_elevatorHigh{[this] {m_elevator.setElevatorPosition(kElevatorHigh); }, {&m_elevator}};
  frc2::InstantCommand m_elevatorShelf{[this] {m_elevator.setElevatorPosition(kElevatorShelf); }, {&m_elevator}};
  frc2::InstantCommand m_toggleColor{[this] {m_elevator.ToggleColor(); }, {&m_elevator}};

  frc2::InstantCommand m_stop{[this] {m_drive.Stop(); }, {&m_drive}};

  autoBalance m_balancing{&m_drive};

  //start of auto commands
  std::unordered_map<std::string, std::shared_ptr<frc2::Command>> eventMap = {
    {"marker1", std::make_shared<frc2::PrintCommand>("Passed marker")},
    {"balance", std::make_shared<autoBalance>(&m_drive)},
    {"wait_1sec", std::make_shared<frc2::WaitCommand>(1.0_s)},
    {"just_intake", std::make_shared<frc2::SequentialCommandGroup>(frc2::WaitCommand(0.5_s),
                                                                   m_stopIntake,
                                                                   m_retractIntake
                                                                  )},
    {"intake_piece", std::make_shared<frc2::SequentialCommandGroup>(m_extendIntake,
                                                                    m_runIntakeOpposite
                                                                    //frc2::WaitCommand(0.2_s),
                                                                    //m_stopIntake,
                                                                    //m_retractIntake
                                                                    )},
    {"score_high", std::make_shared<frc2::SequentialCommandGroup>(m_elevatorHigh, 
                                                                  frc2::WaitCommand(0.7_s),
                                                                  m_extendIntake, 
                                                                  frc2::WaitCommand(0.7_s),
                                                                  m_runIntake, 
                                                                  frc2::WaitCommand(0.5_s),
                                                                  m_stopIntake,
                                                                  m_retractIntake,
                                                                  m_elevatorDown,
                                                                  frc2::WaitCommand(0.4_s))},
    {"score_middle", std::make_shared<frc2::SequentialCommandGroup>(m_elevatorMid, 
                                                                    frc2::WaitCommand(1.0_s),
                                                                    m_extendIntake,
                                                                    frc2::WaitCommand(0.3_s),
                                                                    m_runIntake, 
                                                                    frc2::WaitCommand(0.4_s),
                                                                    m_stopIntake,
                                                                    m_retractIntake,
                                                                    m_elevatorDown,
                                                                    frc2::WaitCommand(0.4_s))},
    {"stop", std::make_shared<frc2::SequentialCommandGroup>(m_stop)}
  };

  pathplanner::SwerveAutoBuilder autoBuilder;
  static std::vector<pathplanner::PathPlannerTrajectory> autoPath1;

  frc2::CommandPtr autoNum1;

//auto path 2
  static std::vector<pathplanner::PathPlannerTrajectory> autoPath2;

  frc2::CommandPtr autoNum2;

  static std::vector<pathplanner::PathPlannerTrajectory> autoPath3;

  frc2::CommandPtr autoNum3;

  static std::vector<pathplanner::PathPlannerTrajectory> autoPath4;

  frc2::CommandPtr autoNum4;

  static std::vector<pathplanner::PathPlannerTrajectory> autoPath5;

  frc2::CommandPtr autoNum5;

  static std::vector<pathplanner::PathPlannerTrajectory> autoPath6;

  frc2::CommandPtr autoNum6;
};
