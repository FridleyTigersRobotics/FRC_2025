// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/AlgaeIntake.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/SparkMax.h>
#include <rev/config/SparkMaxConfig.h>



AlgaeIntake::AlgaeIntake(  Elevator const &Elevator ) :
    m_Elevator{ Elevator }
{
    m_algaeAngleState = AngleStop;
    //m_AlgaeEncoder.SetPosition( 0.0 ); absolute encoder does not have a set function as it is absolute
    m_AlgaeAnglePid.SetIntegratorRange (-0.1, 0.1 ); //stops integrator wind-up
    m_AlgaeAnglePid.SetTolerance(0.001);
    m_AlgaeAnglePid.Reset();

    SparkMaxConfig AlgaeAngleMotorConfig;
    AlgaeAngleMotorConfig
        .Inverted(true)
        .SetIdleMode(SparkMaxConfig::IdleMode::kBrake)
        .VoltageCompensation(12.0)
        .SmartCurrentLimit(15);


    //m_AlgaeAngleMotor.SetInverted(true); this is depreciated
    m_AlgaeAngleMotor.Configure(AlgaeAngleMotorConfig,SparkMax::ResetMode::kResetSafeParameters, SparkMax::PersistMode::kPersistParameters);
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




// This method will be called once per scheduler run
void AlgaeIntake::Periodic() {

    if(m_algaeAngleState == AngleUp)
    {
        m_AlgaeAnglePid.SetSetpoint(Constants::kAlgaeAngleUp);
    }
    if(m_algaeAngleState == AngleDn)
    {
        m_AlgaeAnglePid.SetSetpoint(Constants::kAlgaeAngleDn);
        /*
        if ( m_Elevator.IsNearBottom() )
        {
            // Need to lift up if to close to bottom.
            m_AlgaeAnglePid.SetSetpoint(Constants::kAlgaeAngleUp);
        }
        else
        {
            m_AlgaeAnglePid.SetSetpoint(Constants::kAlgaeAngleDn);
        }
        */
    }

    if(m_algaeAngleState == AngleStop)
    {
        m_AlgaeAngleMotor.Set(0.00);
    }
    else
    {
        double angleMotorValue = m_AlgaeAnglePid.Calculate(m_AlgaeAngleEncoder.Get());
        angleMotorValue = std::clamp(angleMotorValue, -Constants::kAlgaeAngleSpeed, Constants::kAlgaeAngleSpeed );
        m_AlgaeAngleMotor.Set(angleMotorValue);
    }
}




// ****************************************************************************
void AlgaeIntake::TeleopInit()
{
    m_algaeAngleState = AngleStop;
}


// ****************************************************************************
frc2::CommandPtr AlgaeIntake::ChangeStateCommand( AlgaeAngleState_t Astate )
{
    return RunOnce([ this, Astate ] { ChangeState( Astate ); });
}




// ****************************************************************************
void AlgaeIntake::ChangeState( AlgaeAngleState_t Astate )
{
    if(Astate != AngleMaintain)
    {
      m_algaeAngleState = Astate;  
    }
    
}

// ****************************************************************************
void AlgaeIntake::UpdateSmartDashboardData( )
{
    //frc::SmartDashboard::PutNumber("Algae Angle Encoder Value", m_AlgaeAngleEncoder.Get());
    //frc::SmartDashboard::PutNumber("Algae Angle Setpoint", m_AlgaeAnglePid.GetSetpoint());
    //frc::SmartDashboard::PutNumber("Algae Angle motor value", m_AlgaeAngleMotor.Get());
}


