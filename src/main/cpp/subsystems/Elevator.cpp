// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Elevator.h"

Elevator::Elevator()
#ifdef ELEVATOR_VALID
    :encoderMotor1(m_elevatorMotor1.GetEncoder()) 
{
    m_elevatorMotor2.Follow(m_elevatorMotor1, true);
    encoderMotor1.SetPositionConversionFactor(1.121156 / (281.0 / 30.0 * 7.0));
    encoderMotor1.SetPosition(0);
#else
{
#endif

};

// This method will be called once per scheduler run
void Elevator::Periodic() {}
