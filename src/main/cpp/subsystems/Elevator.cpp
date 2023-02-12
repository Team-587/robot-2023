// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Elevator.h"
#include <iostream>

Elevator::Elevator()
#ifdef ELEVATOR_VALID
    :encoderMotor1(m_elevatorMotor1.GetEncoder()) 
{
    m_elevatorMotor2.Follow(m_elevatorMotor1, true);
    encoderMotor1.SetPositionConversionFactor((std::numbers::pi * 1.751)/12.0);
    encoderMotor1.SetPosition(0);
    //rev::SparkMaxLimitSwitch forwardSwitch = m_elevatorMotor1.GetForwardLimitSwitch();
    //forwardSwitch.EnableLimitSwitch(true);

    //rev::SparkMaxLimitSwitch reverseSwitch = m_elevatorMotor1.GetReverseLimitSwitch();
    //reverseSwitch.EnableLimitSwitch(true);

#else
{
#endif

};

void Elevator::setElevatorPosition(double position) {
    std::cout << "Set elevator to " << position << "\n";
    destination = position;
}
// This method will be called once per scheduler run
void Elevator::Periodic() {
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
}
