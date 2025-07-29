// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SpinBoi.h"
#include "Constants.h"

using namespace subsystems;

SpinBoi::SpinBoi() : m_spinBoiMotor{ids::spinBoi}
{
}

// This method will be called once per scheduler run
void SpinBoi::Periodic() {}


void SpinBoi::SetSpeed(double speed)
{
    m_spinBoiMotor.Set(speed);
}
