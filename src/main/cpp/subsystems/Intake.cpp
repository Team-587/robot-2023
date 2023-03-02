// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Intake.h"
#include <iostream>

Intake::Intake() {
    #ifdef INTAKE_VALID
    m_intakeMotor.SetSmartCurrentLimit(30);
    m_intakeMotor.SetSecondaryCurrentLimit(40);
    m_intakeMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    #endif
};

void Intake::checkControl(double wheelSpeed) {
    if (in_auto) return;

    m_speed = wheelSpeed;
}

void Intake::autoSpeed(double speed){
   m_speed = speed;
}

void Intake::setAuto(bool val){
   in_auto = val;
}

void Intake::extended(bool extend) {
    std::cout << "Intake out : " << extend << "\n";
    #ifdef INTAKE_VALID
    if (extend == true){
        m_intakeSolenoid.Set(frc::DoubleSolenoid::kForward);
    } else {
        m_intakeSolenoid.Set(frc::DoubleSolenoid::kReverse);
    }
    #endif
    
}

// This method will be called once per scheduler run
void Intake::Periodic() {
    #ifdef INTAKE_VALID
    //std::cout << "Intake motor : " << m_speed << "\n";
    m_intakeMotor.Set(m_speed);
    #endif
}
