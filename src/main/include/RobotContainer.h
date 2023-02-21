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
#include <pathplanner/lib/PathPlanner.h>
#include <pathplanner/lib/PathConstraints.h>
#include <pathplanner/lib/PathPlannerTrajectory.h>
#include <pathplanner/lib/PathPoint.h>
#include <pathplanner/lib/GeometryUtil.h>
#include <pathplanner/lib/auto/SwerveAutoBuilder.h>
#include <pathplanner/lib/commands/FollowPathWithEvents.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/Trigger.h>

#include "Constants.h"
#include "subsystems/DriveSubsystem.h"
#include "VisionContainer.h"
#include "subsystems/Intake.h"
#include "subsystems/TagVision.h"
#include "commands/autoBalance.h"
#include "subsystems/Elevator.h"
#include "subsystems/PoseEstimatorSubsystem.h"

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

    VisionContainer* GetVisionCone() { return m_pVisionCone; }
    void SetVisionCone(VisionContainer* pVisionCone) { m_pVisionCone = pVisionCone; }
    VisionContainer* GetVisionCube() { return m_pVisionCube; }
    void SetVisionCube(VisionContainer* pVisionCube) { m_pVisionCube = pVisionCube; }

    void StartVision();
    void StopVision();

    photonlib::PhotonCamera m_camera {CameraNames::CAMERA_APRILTAG_FORWARD};
    
    // The robot's subsystems
    DriveSubsystem m_drive;
    Intake m_intake;
    Elevator m_elevator;
    PoseEstimatorSubsystem m_poseEstimator;
    
private:

    // The driver's controller
    frc::XboxController m_driverController{ OIConstants::kDriverControllerPort };
    frc::XboxController m_coDriverController{ OIConstants::kCoDriverControllerPort };
    // The robot's subsystems and commands are defined here...

    // The robot's subsystems
    //DriveSubsystem m_drive;
    //Intake m_intake;

    // The chooser for the autonomous routines
    frc::SendableChooser<frc2::Command*> m_chooser;
    frc2::InstantCommand m_elevatorDown{ [this] {m_elevator.setElevatorPosition(kElevatorDown); }, {&m_elevator} };
    frc2::InstantCommand m_elevatorMid{ [this] {m_elevator.setElevatorPosition(kElevatorMid); }, {&m_elevator} };
    frc2::InstantCommand m_elevatorHigh{ [this] {m_elevator.setElevatorPosition(kElevatorHigh); }, {&m_elevator} };

    autoBalance m_balancing{ &m_drive };

    //start of auto commands
    std::unordered_map<std::string, std::shared_ptr<frc2::Command>> eventMap;
    pathplanner::SwerveAutoBuilder autoBuilder;
    static std::vector<pathplanner::PathPlannerTrajectory> autoPath1;
    
    void ConfigureButtonBindings();

    frc2::InstantCommand m_ZeroHeading{ [this] { m_drive.ZeroHeading(); }, { &m_drive } };
    frc2::InstantCommand m_limitSpeed{ [this] { m_drive.limitSpeed(); }, { &m_drive } };
    frc2::InstantCommand m_fullSpeed{ [this] { m_drive.fullSpeed(); }, { &m_drive } };
    frc2::InstantCommand m_visionAimOn{ [this] { m_drive.setVisionAim(true); }, { &m_drive } };
    frc2::InstantCommand m_visionAimOff{ [this] { m_drive.setVisionAim(false); }, { &m_drive } };
    frc2::InstantCommand m_extendIntake{ [this] {m_intake.extended(true); }, {&m_intake} };
    frc2::InstantCommand m_retractIntake{ [this] {m_intake.extended(false); }, {&m_intake} };

    frc2::Trigger alignCenter{ [this] {return m_driverController.GetYButton(); } };
    frc2::Trigger alignLeft{ [this] {return m_driverController.GetBButton(); } };
    frc2::Trigger alignRight{ [this] {return m_driverController.GetXButton(); } };

    //start of auto commands
    //std::unordered_map<std::string, std::shared_ptr<frc2::Command>> eventMap;
    //pathplanner::SwerveAutoBuilder autoBuilder;
    //static std::vector<pathplanner::PathPlannerTrajectory> autoPath1;

    frc2::CommandPtr autoNum1;

    //auto path 2
    static std::vector<pathplanner::PathPlannerTrajectory> autoPath2;

    frc2::CommandPtr autoNum2;

    static std::vector<pathplanner::PathPlannerTrajectory> autoPath3;

    frc2::CommandPtr autoNum3;

    static std::vector<pathplanner::PathPlannerTrajectory> autoPath4;

    frc2::CommandPtr autoNum4;

    //april tag vision pose estimator
    //TagVision m_tagVision;

    // Vision and camera thread
    VisionContainer* m_pVisionCone;
    VisionContainer* m_pVisionCube;

    //turning pid for vision aim
    frc2::PIDController m_turningController{ .01, 0, .01 };
};
