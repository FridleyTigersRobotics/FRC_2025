// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/CoralIntake.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/SparkMax.h>
#include <rev/config/SparkMaxConfig.h>



CoralIntake::CoralIntake(  Elevator const &Elevator ) :
    m_Elevator{ Elevator }
{
    m_coralAngleState = AngleStop;
    m_coralIntakeState = intakeStop;
    //m_CoralEncoder.SetPosition( 0.0 ); absolute encoder does not have a set function as it is absolute
    m_CoralIntakeEncoder.SetPosition( 0.0 );
    m_CoralAnglePid.SetIntegratorRange (-0.1, 0.1 ); //stops integrator wind-up
    m_CoralAnglePid.SetTolerance(0.001);
    m_CoralAnglePid.Reset();

    SparkMaxConfig CoralAngleMotorConfig;
    CoralAngleMotorConfig.Inverted(true);
    //m_CoralAngleMotor.SetInverted(true); this is depreciated
    m_CoralAngleMotor.Configure(CoralAngleMotorConfig,SparkMax::ResetMode::kResetSafeParameters, SparkMax::PersistMode::kPersistParameters);
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
void CoralIntake::Periodic() {

    if (m_coralIntakeState == intakeIntake)
    {
        m_CoralIntakeMotor.Set(Constants::kCoralIntakeSpeed);
    }

    else if (m_coralIntakeState == intakeReverse)
    {
        m_CoralIntakeMotor.Set(-Constants::kCoralIntakeSpeed);
    }

    else if (m_coralIntakeState == autoIntakeFwd)
    {
        int tofSensVal = m_CoralDetector.GetValue();
        if(tofSensVal>Constants::kTOFtrigger)
        {
            m_CoralIntakeMotor.Set(0.00); 
        }
        else
        {
            m_CoralIntakeMotor.Set(Constants::kCoralIntakeSpeed); 
        }
    }

    else if (m_coralIntakeState == intakeStop)
    {
        m_CoralIntakeMotor.Set(0.00);
    }

    if(m_coralAngleState == AngleUp)
    {
        if(m_Elevator.ismoving())
        {
            m_CoralAnglePid.SetSetpoint(Constants::kCoralAngleVertical);//put the angle down to move, dont allow it when up
        }
        if(!m_Elevator.ismoving())
        {
            m_CoralAnglePid.SetSetpoint(Constants::kCoralAngleUp);
        }
        double angleMotorValue = m_CoralAnglePid.Calculate(m_CoralAngleEncoder.Get());
        angleMotorValue = std::clamp(angleMotorValue, -Constants::kCoralAngleSpeed, Constants::kCoralAngleSpeed );
        m_CoralAngleMotor.Set(angleMotorValue);// motor is reversed
    }
    if(m_coralAngleState == AngleDn)
    {
        m_CoralAnglePid.SetSetpoint(Constants::kCoralAngleDn);
        double angleMotorValue = m_CoralAnglePid.Calculate(m_CoralAngleEncoder.Get());
        angleMotorValue = std::clamp(angleMotorValue, -Constants::kCoralAngleSpeed, Constants::kCoralAngleSpeed );
        m_CoralAngleMotor.Set(angleMotorValue);
    }
    if(m_coralAngleState == AngleStop)
    {
        m_CoralAngleMotor.Set(0.00);
    }
    if(m_coralAngleState == AnglePlaceCoral)
    {
        if(m_Elevator.ismoving())
        {
            m_CoralAnglePid.SetSetpoint(Constants::kCoralAngleVertical);//put the angle down to move, dont allow it when up
        }
        if(!m_Elevator.ismoving())
        {
            m_CoralAnglePid.SetSetpoint(Constants::kCoralAnglePlace);
        }
        double angleMotorValue = m_CoralAnglePid.Calculate(m_CoralAngleEncoder.Get());
        angleMotorValue = std::clamp(angleMotorValue, -Constants::kCoralAngleSpeed, Constants::kCoralAngleSpeed );
        m_CoralAngleMotor.Set(angleMotorValue);
    }
    if(m_coralAngleState == AnglePlaceTopCoral)
    {
        if(m_Elevator.ismoving())
        {
            m_CoralAnglePid.SetSetpoint(Constants::kCoralAngleVertical);//put the angle down to move, dont allow it when up
        }
        if(!m_Elevator.ismoving())
        {
            m_CoralAnglePid.SetSetpoint(Constants::kCoralAngleTopPlace);
        }
        double angleMotorValue = m_CoralAnglePid.Calculate(m_CoralAngleEncoder.Get());
        angleMotorValue = std::clamp(angleMotorValue, -Constants::kCoralAngleSpeed, Constants::kCoralAngleSpeed );
        m_CoralAngleMotor.Set(angleMotorValue);
    }
    if(m_coralAngleState == AngleHorizontal)
    {
        if(m_Elevator.ismoving())
        {
            m_CoralAnglePid.SetSetpoint(Constants::kCoralAngleVertical);//put the angle down to move, dont allow it when up
        }
        if(!m_Elevator.ismoving())
        {
            m_CoralAnglePid.SetSetpoint(Constants::kCoralAngleHorizontal);
        }
        double angleMotorValue = m_CoralAnglePid.Calculate(m_CoralAngleEncoder.Get());
        angleMotorValue = std::clamp(angleMotorValue, -Constants::kCoralAngleSpeed, Constants::kCoralAngleSpeed );
        m_CoralAngleMotor.Set(angleMotorValue);
    }
}




// ****************************************************************************
void CoralIntake::TeleopInit()
{
    m_coralAngleState = AngleStop;
    m_coralIntakeState = intakeStop;
}


// ****************************************************************************
frc2::CommandPtr CoralIntake::ChangeStateCommand( CoralAngleState_t Astate, CoralIntakeState_t Istate )
{
    return RunOnce([ this, Astate, Istate ] { ChangeState( Astate, Istate ); });
}




// ****************************************************************************
void CoralIntake::ChangeState( CoralAngleState_t Astate, CoralIntakeState_t Istate )
{
    if(Astate != AngleMaintain)
    {
      m_coralAngleState = Astate;  
    }
    if(Istate != intakeMaintain)
    {
      m_coralIntakeState = Istate; 
    }
    
}

// ****************************************************************************
void CoralIntake::UpdateSmartDashboardData( )
{
    frc::SmartDashboard::PutNumber("Coral Angle Encoder Value", m_CoralAngleEncoder.Get());
    frc::SmartDashboard::PutNumber("Coral Angle Setpoint", m_CoralAnglePid.GetSetpoint());
    frc::SmartDashboard::PutNumber("Coral Intake Motor Encoder Value", m_CoralIntakeEncoder.GetPosition());
    frc::SmartDashboard::PutNumber("Coral Angle motor value", m_CoralAngleMotor.Get());
    frc::SmartDashboard::PutNumber("Coral TOF Sensor Value", m_CoralDetector.GetValue());
}


