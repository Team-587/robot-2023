// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Elevator.h"
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/XboxController.h>


Elevator::Elevator():
    color1(0),
    color2(1)
#ifdef ELEVATOR_VALID
    ,encoderMotor1(m_elevatorMotor1.GetEncoder()),
    PID_motor1(m_elevatorMotor1.GetPIDController())
#endif
{
    #ifdef ELEVATOR_VALID
    PID_motor1.SetFeedbackDevice(encoderMotor1);
    m_elevatorMotor2.Follow(m_elevatorMotor1, true);
    encoderMotor1.SetPositionConversionFactor((std::numbers::pi * 1.751)/12.0);
    encoderMotor1.SetPosition(0);

    PID_motor1.SetP(.1);
    PID_motor1.SetI(0);
    PID_motor1.SetD(0);
    PID_motor1.SetFF(0);
    PID_motor1.SetOutputRange(-0.25, 0.25);
    m_elevatorMotor1.SetOpenLoopRampRate(0.5);
    m_elevatorMotor2.SetOpenLoopRampRate(0.5);
    m_elevatorMotor1.SetSmartCurrentLimit(60);
    m_elevatorMotor2.SetSmartCurrentLimit(60);
    m_elevatorMotor1.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, kElevatorMax);
    m_elevatorMotor1.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, kElevatorDown);
    m_elevatorMotor1.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_elevatorMotor2.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

    frc::SmartDashboard::PutData("ElevatorPID", &elevatorPID);
    rev::SparkMaxLimitSwitch forwardSwitch = m_elevatorMotor1.GetForwardLimitSwitch(rev::SparkMaxLimitSwitch::Type::kNormallyOpen);
    forwardSwitch.EnableLimitSwitch(true);

    rev::SparkMaxLimitSwitch reverseSwitch = m_elevatorMotor1.GetReverseLimitSwitch(rev::SparkMaxLimitSwitch::Type::kNormallyOpen);
    reverseSwitch.EnableLimitSwitch(true);

    #endif

    color1.Set(true);
    color2.Set(true);
};
void Elevator::ToggleColor() {
    currentColor++;
    if (currentColor > 2) {
        currentColor = 0;
    }
    if (currentColor == 0 ) {
        color1.Set(false);
        color2.Set(false);
    } else if ( currentColor == 1) {
        color1.Set(true);
        color2.Set(false);
    } else if (currentColor == 2) {
        color2.Set(true);
        color1.Set(false);
    } 
    
}
void Elevator::setElevatorPosition(double position) {
    std::cout << "Set elevator to " << position << "\n";
    destination = position;
    #ifdef ELEVATOR_VALID
    PID_motor1.SetReference(destination, rev::CANSparkMax::ControlType::kPosition);
    #endif
}
// This method will be called once per scheduler run
void Elevator::Periodic() {
  frc::XboxController m_coDriverController{OIConstants::kCoDriverControllerPort};
  double manualElevator = m_coDriverController.GetRightY();

  if (fabs(manualElevator) > 0.25) {
    destination = destination - (manualElevator * 0.1);
    setElevatorPosition(destination);
  }

  //frc::SmartDashboard::PutNumber("Elev. Destination", destination);
  #ifdef ELEVATOR_VALID
  //frc::SmartDashboard::PutNumber("Elev. Position", encoderMotor1.GetPosition());   
  #endif

  if (P != elevatorPID.GetP() || I != elevatorPID.GetI() || D != elevatorPID.GetD()) {
    P = elevatorPID.GetP();
    I = elevatorPID.GetI();
    D = elevatorPID.GetD();

    #ifdef ELEVATOR_VALID
    PID_motor1.SetP(P);
    PID_motor1.SetI(I);
    PID_motor1.SetD(D);
    #endif
  }
    

    /*
    double position = 0.0;
    #ifdef ELEVATOR_VALID
    position = encoderMotor1.GetPosition();
    #endif
    
    auto driveOutput = elevatorPID.Calculate(position, destination);
    if (driveOutput > 1.0){
        driveOutput = 1.0;

    } else if (driveOutput < -1.0) {
        driveOutput = -1.0;
    }
    #ifdef ELEVATOR_VALID
    m_elevatorMotor1.Set(driveOutput);
    #endif
    */

}
