// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <numbers>
#include <frc/ADXRS450_Gyro.h>
#include <frc/Encoder.h>
#include <frc/drive/MecanumDrive.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/interfaces/Gyro.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <frc2/command/SubsystemBase.h>
#include <units/voltage.h>
#include <units/angular_velocity.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/acceleration.h>
#include <AHRS.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>

#include "Constants.h"
#include "SwerveModule.h"
#include "VisionContainer.h"
#include "subsystems/TagVision.h"

class DriveSubsystem : public frc2::SubsystemBase
{
public:
    DriveSubsystem();

    //TagVision m_tagVision;

    /**
     * Will be called periodically whenever the CommandScheduler runs.
     */
    void Periodic() override;

    // Subsystem methods go here.

    /**
     * Drives the robot at given x, y and theta speeds. Speeds range from [-1, 1]
     * and the linear speeds have no effect on the angular speed.
     *
     * @param xSpeed        Speed of the robot in the x direction
     *                      (forward/backwards).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to
     *                      the field.
     */
    void Drive(units::meters_per_second_t xSpeed,
               units::meters_per_second_t ySpeed, units::radians_per_second_t rot,
               bool fieldRelative);

    /**
     * Resets the drive encoders to currently read a position of 0.
     */
    void ResetEncoders();

    /**
     * Sets the drive MotorControllers to a power from -1 to 1.
     */
    void SetModuleStates(wpi::array<frc::SwerveModuleState, 4> desiredStates);

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from 180 to 180
     */
    units::degree_t GetHeading() const;

    /**
     * Zeroes the heading of the robot.
     */
    void ZeroHeading();

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    double GetTurnRate();
    
    void limitSpeed();

    void fullSpeed();
    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    frc::Pose2d GetPose();

    //sets vision measurements
    void visionMeasurements(frc::Pose2d pose, units::second_t timestamp);

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    void ResetOdometry(frc::Pose2d pose);

    units::meter_t kTrackWidth =
        0.59_m; // Distance between centers of right and left wheels on robot
    units::meter_t kWheelBase =
        0.59_m; // Distance between centers of front and back wheels on robot

    frc::SwerveDriveKinematics<4> kDriveKinematics{
        frc::Translation2d(kWheelBase / 2, kTrackWidth / 2),
        frc::Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        frc::Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        frc::Translation2d(-kWheelBase / 2, -kTrackWidth / 2)};

     void setVisionAim(bool visionAim) {
        m_visionAim = visionAim;
    }

    bool getVisionAim() {
        return m_visionAim;
    }

private:
    // Components (e.g. motor controllers and sensors) should generally be
    // declared private and exposed only through public methods.

    SwerveModule m_frontLeft;
    SwerveModule m_rearLeft;
    SwerveModule m_frontRight;
    SwerveModule m_rearRight;

    double m_fullSpeed = 1.0;

    std::array<frc::SwerveModulePosition, 4> odometryPos {
        m_frontLeft.GetPosition(),
        m_rearLeft.GetPosition(),
        m_frontRight.GetPosition(),
        m_rearRight.GetPosition()
    };

    // The gyro sensor
    //frc::ADXRS450_Gyro m_gyro;
    //AHRS m_NavX;
    AHRS m_NavX{frc::SPI::Port::kMXP};
    
    // Odometry class for tracking robot pose
    // 4 defines the number of modules
    frc::SwerveDriveOdometry<4> m_odometry;
    frc::SwerveDrivePoseEstimator<4> m_poseEstimator {
        kDriveKinematics,
        frc::Rotation2d(),
        odometryPos,
        frc::Pose2d()
    };
    bool m_visionAim;
    
};
