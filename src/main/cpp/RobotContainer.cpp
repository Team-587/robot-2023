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
#include <frc2/command/WaitCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <frc2/command/button/JoystickButton.h>
#include <units/angle.h>
#include <units/velocity.h>
#include <math.h>
#include "Constants.h"
#include "subsystems/DriveSubsystem.h"

using namespace DriveConstants;
using namespace pathplanner;

std::vector<PathPlannerTrajectory> RobotContainer::autoPath1 = PathPlanner::loadPathGroup("auto1", {PathConstraints(3.0_mps, 3.0_mps_sq)});
std::vector<PathPlannerTrajectory> RobotContainer::autoPath2 = PathPlanner::loadPathGroup("auto2", {PathConstraints(3.0_mps, 3.0_mps_sq)});
std::vector<PathPlannerTrajectory> RobotContainer::autoPath3 = PathPlanner::loadPathGroup("auto3", {PathConstraints(3.0_mps, 3.0_mps_sq)});//, PathConstraints(1.3_mps, 1.3_mps_sq), PathConstraints(3.0_mps, 3.0_mps_sq), PathConstraints(3.0_mps, 3.0_mps_sq)});
std::vector<PathPlannerTrajectory> RobotContainer::autoPath4 = PathPlanner::loadPathGroup("auto4", {PathConstraints(3.0_mps, 3.0_mps_sq)});
std::vector<PathPlannerTrajectory> RobotContainer::autoPath5 = PathPlanner::loadPathGroup("auto5", {PathConstraints(1.0_mps, 1.0_mps_sq),PathConstraints(1.0_mps, 1.0_mps_sq),PathConstraints(2.0_mps, 2.0_mps_sq)});
std::vector<PathPlannerTrajectory> RobotContainer::autoPath6 = PathPlanner::loadPathGroup("auto6", {PathConstraints(3.0_mps, 3.0_mps_sq)});
std::vector<PathPlannerTrajectory> RobotContainer::autoPath7 = PathPlanner::loadPathGroup("Tims Path", {PathConstraints(3.0_mps, 3.0_mps_sq)});

RobotContainer::RobotContainer(): 
  // Initialize all of your commands and subsystems here
    autoBuilder(
      [this]() { return m_drive.GetPose(); }, // Function to supply current robot pose
      [this](auto initPose) { m_drive.ResetOdometry(initPose); }, // Function used to reset odometry at the beginning of auto
      m_drive.kDriveKinematics,
      PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
      PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
      [this](auto speeds) { m_drive.SetModuleStates(speeds); }, // Output function that accepts field relative ChassisSpeeds
      eventMap, // Our event map
      {&m_drive }, // Drive requirements, usually just a single drive subsystem
      true // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
   ),
   autoNum1(autoBuilder.fullAuto(autoPath1)), 
   autoNum2(autoBuilder.fullAuto(autoPath2)),
   autoNum3(autoBuilder.fullAuto(autoPath3)),
   autoNum4(autoBuilder.fullAuto(autoPath4)),
   autoNum5(autoBuilder.fullAuto(autoPath5)),
   autoNum6(autoBuilder.fullAuto(autoPath6)),
   autoNum7(autoBuilder.fullAuto(autoPath7))

{

  //eventMap.emplace("marker1", std::make_shared<frc2::PrintCommand>("Passed BAlance1"));
  //eventMap.emplace("balance", std::make_shared<autoBalance>(&m_drive));
  //eventMap.emplace("wait_1sec", std::make_shared<frc2::WaitCommand>(1.0_s));
  //eventMap.emplace("extend_intake", std::make_shared<frc2::SequentialCommandGroup>(m_elevatorHigh, m_extendIntake));

    m_chooser.SetDefaultOption("Wall Score", autoNum2.get());
    m_chooser.AddOption("Wall Charge Station", autoNum1.get());
    m_chooser.AddOption("HP Score", autoNum3.get());
    m_chooser.AddOption("HP Charge Station", autoNum4.get());
    m_chooser.AddOption("Center Start", autoNum5.get());
    m_chooser.AddOption("Auto Path 6", autoNum6.get());
    m_chooser.AddOption("Tims magic", autoNum7.get());

   frc::SmartDashboard::PutData(&m_chooser);
  // Configure the button bindings
  ConfigureButtonBindings();

  // Set up default drive command
  // The left stick controls translation of the robot.
  // Turning is controlled by the X axis of the right stick.
  m_drive.SetDefaultCommand(frc2::RunCommand(
      [this] {

        double rotation = m_driverController.GetRightX();
        double currentYaw = m_driverController.getCurrentYaw();

        if (m_driverController.GetRawButton(frc::XboxController::Button::kY)) {
          rotation = headerPID.Calculate(currentYaw, 0);

        } else if (m_driverController.GetRawButton(frc::XboxController::Button::kB)) {
          rotation = headerPID.Calculate(currentYaw, -90);

        } else if (m_driverController.GetRawButton(frc::XboxController::Button::kA)) {
          rotation = headerPID.Calculate(currentYaw, 180);

        } else if (m_driverController.GetRawButton(frc::XboxController::Button::kX)) {
          rotation = headerPID.Calculate(currentYaw, 90);
        }

        m_drive.Drive(
            -units::meters_per_second_t(m_driverController.GetLeftY()),
            -units::meters_per_second_t(m_driverController.GetLeftX()),
            -units::radians_per_second_t(rotation), true);
      },
      {&m_drive}));


  m_intake.SetDefaultCommand(frc2::RunCommand(
    [this] {
      if(abs(m_coDriverController.GetLeftY()) < .1) {
        m_intake.checkControl(0);
      } else {
        m_intake.checkControl(m_coDriverController.GetLeftY());
      }
    },{&m_intake}));

}

void RobotContainer::ConfigureButtonBindings() {
//Co driver buttons
    frc2::JoystickButton extendIntakeButton{&m_coDriverController, frc::XboxController::Button::kLeftBumper};
    extendIntakeButton.OnTrue(&m_retractIntake);
    frc2::JoystickButton retractIntakeButton{&m_coDriverController, frc::XboxController::Button::kRightBumper};
    retractIntakeButton.OnTrue(&m_extendIntake);
    frc2::JoystickButton elevatorDownButton{&m_coDriverController, frc::XboxController::Button::kA};
    elevatorDownButton.OnTrue(&m_elevatorDown);
    frc2::JoystickButton elevatorLowButton{&m_coDriverController, frc::XboxController::Button::kX};
    elevatorLowButton.OnTrue(&m_elevatorLow);
    frc2::JoystickButton elevatorMidButton{&m_coDriverController, frc::XboxController::Button::kB};
    elevatorMidButton.OnTrue(&m_elevatorMid);
    frc2::JoystickButton elevatorHighButton{&m_coDriverController, frc::XboxController::Button::kY};
    elevatorHighButton.OnTrue(&m_elevatorHigh);
    frc2::JoystickButton toggleColorButton{&m_coDriverController, frc::XboxController::Button::kBack};
    toggleColorButton.OnTrue(&m_toggleColor);
    frc2::JoystickButton toggleElevatorShelf{&m_coDriverController, frc::XboxController::Button::kStart};
    toggleElevatorShelf.OnTrue(&m_elevatorShelf);
    //frc2::JoystickButton zeroElevatorHeading{&m_coDriverController, frc::XboxController::Button::kRightStick};

    //frc2::JoystickButton intakeRunButton{&m_driverController, frc::XboxController::Button::kX};
    //intakeRunButton.OnTrue(&m_runIntake);
    //frc2::JoystickButton intakeStopButton{&m_driverController, frc::XboxController::Button::kA};
    //intakeStopButton.OnTrue(&m_stopIntake);
    //frc2::JoystickButton intakeReverseButton{&m_driverController, frc::XboxController::Button::kB};
    //intakeReverseButton.OnTrue(&m_runIntakeOpposite);




    //Driver buttons
    frc2::JoystickButton startButton{&m_driverController, frc::XboxController::Button::kStart};
    startButton.OnTrue(&m_ZeroHeading);
    frc2::JoystickButton LeftBumper{&m_driverController, frc::XboxController::Button::kLeftBumper};
    LeftBumper.OnTrue(&m_quarterSpeed).OnFalse(&m_fullSpeed);
    frc2::JoystickButton RightBumper{&m_driverController, frc::XboxController::Button::kRightBumper};
    RightBumper.OnTrue(&m_halfSpeed).OnFalse(&m_fullSpeed);
    /*frc2::JoystickButton balancingButton{&m_driverController, frc::XboxController::Button::kY};
    frc2::JoystickButton elevatorDownButtondriver{&m_driverController, frc::XboxController::Button::kA};
    elevatorDownButtondriver.OnTrue(&m_elevatorDown);
    frc2::JoystickButton elevatorAndIntakeInitialStateButtondriver{&m_driverController, frc::XboxController::Button::kX};
    elevatorAndIntakeInitialStateButtondriver.OnTrue(&m_elevatorDown);
    elevatorAndIntakeInitialStateButtondriver.OnTrue(&m_retractIntake);
    //balancingButton.ToggleOnTrue(&m_balancing);
    balancingButton.WhileTrue(&m_balancing);
    */
}

frc2::Command* RobotContainer::GetAutonomousCommand() {

 return m_chooser.GetSelected();
}
