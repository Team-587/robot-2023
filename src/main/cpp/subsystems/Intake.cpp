// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Intake.h"
#include <frc/XboxController.h>
#include <iostream>
#include "Constants.h"

Intake::Intake() {
    #ifdef INTAKE_VALID
    m_intakeMotor.SetSmartCurrentLimit(50);
    m_intakeMotor.SetSecondaryCurrentLimit(80);
    m_intakeMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_intakeMotor.SetInverted(true);
    #endif
};

void Intake::checkControl(double wheelSpeed) {
    if (in_auto) return;
    frc::XboxController m_coDriverController{OIConstants::kCoDriverControllerPort};

    if(m_coDriverController.GetRawAxis(3) > 0.5) {
        m_speed = -0.1;
    } else {
        m_speed = wheelSpeed;
    }
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
