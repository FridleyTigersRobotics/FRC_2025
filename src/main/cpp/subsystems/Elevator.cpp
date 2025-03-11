// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Elevator.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandPtr.h>
#include <rev/SparkMax.h>
#include <rev/config/SparkMaxConfig.h>
using namespace rev::spark;

bool elevatormoving = false;

bool Elevator::ismoving() const
{
    return elevatormoving;
}

bool Elevator::IsNearBottom() const
{
    return m_Elevator0Encoder.GetPosition() < Constants::kElevatorLowestAlgae;
}



// ****************************************************************************
Elevator::Elevator()
{
    m_ElevatorState = ElevatorStop;
    m_Elevator0Encoder.SetPosition( 0.0 );
    m_Elevator1Encoder.SetPosition( 0.0 );
    m_ElevatorPid.SetTolerance(0.001);
    m_ElevatorPid.Reset();

    SparkMaxConfig Motor0Config;
    Motor0Config
    .Inverted(false)
    //ELI BOZO
    .SmartCurrentLimit(25)
    .VoltageCompensation(12);

    SparkMaxConfig Motor1Config;
    Motor1Config
    .Inverted(true)
    //ELI BOZO
    .SmartCurrentLimit(25)
    .VoltageCompensation(12);
    //m_Motor1.SetInverted(true); this is depreciated

    m_Motor1.Configure(Motor1Config, SparkMax::ResetMode::kResetSafeParameters, SparkMax::PersistMode::kPersistParameters);
    m_Motor0.Configure(Motor0Config, SparkMax::ResetMode::kResetSafeParameters, SparkMax::PersistMode::kPersistParameters);
    /*
   * Apply the configuration to the SPARKs.
   *
   * kResetSafeParameters is used to get the SPARK MAX to a known state. This
   * is useful in case the SPARK MAX is replaced.
   *
   * kPersistParameters is used to ensure the configuration is not lost when
   * the SPARK MAX loses power. This is useful for power cycles that may occur
   * mid-operation.
   * https://github.com/REVrobotics/REVLib-Examples/blob/main/C%2B%2B/SPARK/Open%20Loop%20Arcade%20Drive/src/main/cpp/Robot.cpp
   */
}

// ****************************************************************************
void Elevator::TeleopInit()
{
    m_ElevatorState = ElevatorStop;
}


// ****************************************************************************
void Elevator::Periodic()
{
    if(m_ElevatorState == ElevatorStop)
    {
        m_Motor0.Set(0.00);
        m_Motor1.Set(0.00);
    }
    if(m_ElevatorState == ElevatorStartingConfig)
    {
        m_ElevatorPid.SetSetpoint(Constants::kPosStart);
        double elevatorMotorValue = m_ElevatorPid.Calculate(m_Elevator0Encoder.GetPosition());
        elevatorMotorValue = std::clamp( elevatorMotorValue, -Constants::kElevatorSpeed, Constants::kElevatorSpeed );
        m_Motor0.Set(elevatorMotorValue);
        m_Motor1.Set(elevatorMotorValue);
    }
    if(m_ElevatorState == ElevatorCoralL1)
    {
        m_ElevatorPid.SetSetpoint(Constants::kPosCoralL1);
        double elevatorMotorValue = m_ElevatorPid.Calculate(m_Elevator0Encoder.GetPosition());
        elevatorMotorValue = std::clamp( elevatorMotorValue, -Constants::kElevatorSpeed, Constants::kElevatorSpeed );
        m_Motor0.Set(elevatorMotorValue);
        m_Motor1.Set(elevatorMotorValue);
    }
    if(m_ElevatorState == ElevatorCoralL2)
    {
        m_ElevatorPid.SetSetpoint(Constants::kPosCoralL2);
        double elevatorMotorValue = m_ElevatorPid.Calculate(m_Elevator0Encoder.GetPosition());
        elevatorMotorValue = std::clamp( elevatorMotorValue, -Constants::kElevatorSpeed, Constants::kElevatorSpeed );
        m_Motor0.Set(elevatorMotorValue);
        m_Motor1.Set(elevatorMotorValue);
    }
    if(m_ElevatorState == ElevatorCoralL3)
    {
        m_ElevatorPid.SetSetpoint(Constants::kPosCoralL3);
        double elevatorMotorValue = m_ElevatorPid.Calculate(m_Elevator0Encoder.GetPosition());
        elevatorMotorValue = std::clamp( elevatorMotorValue, -Constants::kElevatorSpeed, Constants::kElevatorSpeed );
        m_Motor0.Set(elevatorMotorValue);
        m_Motor1.Set(elevatorMotorValue);
    }
    if(m_ElevatorState == ElevatorCoralL4)
    {
        m_ElevatorPid.SetSetpoint(Constants::kPosCoralL4);
        double elevatorMotorValue = m_ElevatorPid.Calculate(m_Elevator0Encoder.GetPosition());
        elevatorMotorValue = std::clamp( elevatorMotorValue, -Constants::kElevatorSpeed, Constants::kElevatorSpeed );
        m_Motor0.Set(elevatorMotorValue);
        m_Motor1.Set(elevatorMotorValue);
    }
    if(m_ElevatorState == ElevatorCoralIntake)
    {
        m_ElevatorPid.SetSetpoint(Constants::kPosCoralIntake);
        double elevatorMotorValue = m_ElevatorPid.Calculate(m_Elevator0Encoder.GetPosition());
        elevatorMotorValue = std::clamp( elevatorMotorValue, -Constants::kElevatorSpeed, Constants::kElevatorSpeed );
        m_Motor0.Set(elevatorMotorValue);
        m_Motor1.Set(elevatorMotorValue);
    }
    if(m_ElevatorState == ElevatorL2PrepBump)//prepare to bump coral off of L2
    {
        m_ElevatorPid.SetSetpoint(Constants::kPosAlgaeL2PrepBump);
        double elevatorMotorValue = m_ElevatorPid.Calculate(m_Elevator0Encoder.GetPosition());
        elevatorMotorValue = std::clamp( elevatorMotorValue, -Constants::kElevatorSpeed, Constants::kElevatorSpeed );
        m_Motor0.Set(elevatorMotorValue);
        m_Motor1.Set(elevatorMotorValue);
    }
    if(m_ElevatorState == ElevatorL2EndBump)//reached top of L2 bump
    {
        m_ElevatorPid.SetSetpoint(Constants::kPosAlgaeL2EndBump);
        double elevatorMotorValue = m_ElevatorPid.Calculate(m_Elevator0Encoder.GetPosition());
        elevatorMotorValue = std::clamp( elevatorMotorValue, -Constants::kElevatorSpeed, Constants::kElevatorSpeed );
        m_Motor0.Set(elevatorMotorValue);
        m_Motor1.Set(elevatorMotorValue);
    }
    if(m_ElevatorState == ElevatorL3PrepBump)//prepare to bump coral off of L3
    {
        m_ElevatorPid.SetSetpoint(Constants::kPosAlgaeL3PrepBump);
        double elevatorMotorValue = m_ElevatorPid.Calculate(m_Elevator0Encoder.GetPosition());
        elevatorMotorValue = std::clamp( elevatorMotorValue, -Constants::kElevatorSpeed, Constants::kElevatorSpeed );
        m_Motor0.Set(elevatorMotorValue);
        m_Motor1.Set(elevatorMotorValue);
    }
    if(m_ElevatorState == ElevatorL3EndBump)//reached top of L3 bump
    {
        m_ElevatorPid.SetSetpoint(Constants::kPosAlgaeL3EndBump);
        double elevatorMotorValue = m_ElevatorPid.Calculate(m_Elevator0Encoder.GetPosition());
        elevatorMotorValue = std::clamp( elevatorMotorValue, -Constants::kElevatorSpeed, Constants::kElevatorSpeed );
        m_Motor0.Set(elevatorMotorValue);
        m_Motor1.Set(elevatorMotorValue);
    }
    if(fabs(m_Motor0.Get())>0.1 || fabs(m_Motor1.Get())>0.1) //0.1 is the PID I max value (anti-windup)
    {
        elevatormoving = true;
    }
    else if(fabs(m_Motor0.Get())<=0.1 || fabs(m_Motor1.Get())<=0.1)
    {
       elevatormoving = false;
    }
}



// ****************************************************************************
frc2::CommandPtr Elevator::ChangeStateCommand( ElevatorState_t ElevatorPosition )
{
    return RunOnce([ this, ElevatorPosition ] { ChangeState( ElevatorPosition ); });
}



// ****************************************************************************
void Elevator::ChangeState( ElevatorState_t ElevatorPosition )
{
    if(ElevatorPosition != ElevatorMaintain)
    {
        m_ElevatorState = ElevatorPosition;
    }
}


// ****************************************************************************
void Elevator::UpdateSmartDashboardData( )
{
    //frc::SmartDashboard::PutNumber("Elevator Encoder Value", m_Elevator0Encoder.GetPosition());
    //frc::SmartDashboard::PutNumber("Elevator  Setpoint", m_ElevatorPid.GetSetpoint());
    //frc::SmartDashboard::PutNumber("Elevator Motor Value", m_Motor0.Get());
    //frc::SmartDashboard::PutBoolean("Elevator Moving", elevatormoving);

}


// ****************************************************************************
void Elevator::ManualControl()
{
 
}

