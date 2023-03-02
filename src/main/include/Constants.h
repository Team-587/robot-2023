// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#include <numbers>

#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <units/angular_acceleration.h>

#pragma once

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or bool constants.  This should not be used for any other purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */
constexpr double kElevatorDown = 0.0;
constexpr double kElevatorLow = 3.0;
constexpr double kElevatorMid = 18.5;
constexpr double kElevatorHigh = 26.0;
constexpr double kElevatorShelf = 19.0;
constexpr double kElevatorMax = kElevatorHigh + 1.0;


enum VisionPipelineIndex {
    APRILTAG = 0,
    CONE = 10,
    CUBE = 11,
    REFLECTIVE_GREEN = 1,
    REFLECTIVE_RED = 14
};

constexpr int kIntakeSolenoid = 0;
constexpr int kIntakeSolenoid1 = 1;

namespace DriveConstants {
    constexpr int kFrontLeftDriveMotorPort = 11;
    constexpr int kRearLeftDriveMotorPort = 13;
    constexpr int kFrontRightDriveMotorPort = 15;
    constexpr int kRearRightDriveMotorPort = 17;

    constexpr int kFrontLeftTurningMotorPort = 12;
    constexpr int kRearLeftTurningMotorPort = 14;
    constexpr int kFrontRightTurningMotorPort = 16;
    constexpr int kRearRightTurningMotorPort = 18;

    constexpr int kFrontLeftAbsoluteEncoderPort = 21;
    constexpr int kFrontRightAbsoluteEncoderPort = 23;
    constexpr int kRearLeftAbsoluteEncoderPort = 22;
    constexpr int kRearRightAbsoluteEncoderPort = 24;

    constexpr int kIntakeMotor = 30;

    constexpr int kElevatorMotor1 = 31;
    constexpr int kElevatorMotor2 = 32;

    constexpr int kFrontLeftTurningEncoderPorts[2]{ 0, 1 };
    constexpr int kRearLeftTurningEncoderPorts[2]{ 2, 3 };
    constexpr int kFrontRightTurningEncoderPorts[2]{ 4, 5 };
    constexpr int kRearRightTurningEncoderPorts[2]{ 6, 7 };

    constexpr bool kFrontLeftTurningEncoderReversed = false;
    constexpr bool kRearLeftTurningEncoderReversed = false;
    constexpr bool kFrontRightTurningEncoderReversed = false;
    constexpr bool kRearRightTurningEncoderReversed = false;

    constexpr int kFrontLeftDriveEncoderPorts[2]{ 8, 9 };
    constexpr int kRearLeftDriveEncoderPorts[2]{ 10, 11 };
    constexpr int kFrontRightDriveEncoderPorts[2]{ 12, 13 };
    constexpr int kRearRightDriveEncoderPorts[2]{ 14, 15 };

    constexpr bool kFrontLeftDriveEncoderReversed = false;
    constexpr bool kRearLeftDriveEncoderReversed = false;
    constexpr bool kFrontRightDriveEncoderReversed = true;
    constexpr bool kRearRightDriveEncoderReversed = true;

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or
    // theoretically for *your* robot's drive. The SysId tool provides a convenient
    // method for obtaining these values for your robot.
    constexpr auto ks = 1_V;
    constexpr auto kv = 0.8 * 1_V * 1_s / 1_m;
    constexpr auto ka = 0.15 * 1_V * 1_s * 1_s / 1_m;

    // Example value only - as above, this must be tuned for your drive!
    constexpr double kPFrontLeftVel = 0.5;
    constexpr double kPRearLeftVel = 0.5;
    constexpr double kPFrontRightVel = 0.5;
    constexpr double kPRearRightVel = 0.5;
}  // namespace DriveConstants

namespace ModuleConstants {
    constexpr int kEncoderCPR = 1024;
    constexpr double kWheelDiameterMeters = 0.15;
    constexpr double kDriveEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        (kWheelDiameterMeters * std::numbers::pi) /
        static_cast<double>(kEncoderCPR);

    constexpr double kTurningEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        (std::numbers::pi * 2) / static_cast<double>(kEncoderCPR);

    constexpr double kPModuleTurningController = 1;
    constexpr double kPModuleDriveController = 1;
}  // namespace ModuleConstants

namespace AutoConstants {
    constexpr auto kMaxSpeed = 3_mps;
    constexpr auto kMaxAcceleration = 3_mps_sq;
    constexpr auto kMaxAngularSpeed = 3.142_rad_per_s;
    constexpr auto kMaxAngularAcceleration = 3.142_rad_per_s_sq;

    constexpr double kPXController = 0.5;
    constexpr double kPYController = 0.5;
    constexpr double kPThetaController = 0.5;

    //

    extern const frc::TrapezoidProfile<units::radians>::Constraints
        kThetaControllerConstraints;

}  // namespace AutoConstants

namespace OIConstants {
    constexpr int kDriverControllerPort = 0;
    constexpr int kCoDriverControllerPort = 1;
}  // namespace OIConstants

//Camera Constants
namespace Camerapos {
    constexpr units::meter_t cam_height_meters = 11_in; //Value for FreeFall
    //constexpr auto goal_height_meters = units::meter_t{  }; //Value for FreeFall
    constexpr units::radian_t cam_angle_degrees = 20_deg;

    constexpr units::meter_t CAMERA_APRILTAG_FORWARD_X = 0_in;
    constexpr units::meter_t CAMERA_APRILTAG_FORWARD_Y = 0_in;
    constexpr units::meter_t CAMERA_APRILTAG_FORWARD_Z = 11_in;

}

namespace CameraNames {
    const std::string CAMERA_1 = "HD_Pro_Webcam_C920";
    const std::string CAMERA_2 = "OV5647";
    const std::string CAMERA_APRILTAG_FORWARD = "HD_Pro_Webcam_C920";
}

namespace FieldElementsMeasurement {
    const units::meter_t cone_height_centimeters = 33_cm;
    const units::meter_t cone_width_centimeters = 21_cm;
    const units::meter_t cube_width_centimeters = 24_cm; /*"may not be actual dimensions - Game Manual"*/
    const units::meter_t april_tag_height = 18_in;
}
