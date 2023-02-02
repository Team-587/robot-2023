// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Intake.h"

Intake::Intake() = default;

void Intake::checkControl(double wheelSpeed) {
    #ifdef INTAKE_VALID
    m_intakeMotor.Set(wheelSpeed);
    #endif
}

void Intake::extended(bool extend) {
    #ifdef INTAKE_VALID
    if (extend == true){
        m_intakeSolenoid.Set(frc::DoubleSolenoid::kForward);
    } else {
        m_intakeSolenoid.Set(frc::DoubleSolenoid::kReverse);
    }
    #endif
    
}

// This method will be called once per scheduler run
void Intake::Periodic() {}
