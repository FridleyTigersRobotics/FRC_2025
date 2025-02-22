#include <Debug.h>
#include <Claw.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/SparkMax.h>
#include <rev/config/SparkMaxConfig.h>


// ****************************************************************************
void Claw::Init()
{
    m_coralAngleState = AngleStop;
    m_coralIntakeState = intakeStop;
    //m_CoralEncoder.SetPosition( 0.0 ); absolute encoder does not have a set function as it is absolute
    m_CoralIntakeEncoder.SetPosition( 0.0 );
    m_CoralAnglePid.SetIntegratorRange (-0.1, 0.1 ); //stops integrator wind-up
    m_CoralAnglePid.SetTolerance(0.001);
    m_CoralAnglePid.Reset();
    m_CoralAngleMotor.SetInverted(true);

}

// ****************************************************************************
void Claw::TeleopInit()
{
    m_coralAngleState = AngleStop;
    m_coralIntakeState = intakeStop;
}

// ****************************************************************************
void Claw::ManualControl( )
{

}


// ****************************************************************************
void Claw::Update()
{
    if (m_coralIntakeState == intakeIntake)
    {
        m_CoralIntakeMotor.Set(Constants::kIntakeSpeed);
    }

    else if (m_coralIntakeState == intakeReverse)
    {
        m_CoralIntakeMotor.Set(-Constants::kIntakeSpeed);
    }

    else if (m_coralIntakeState == intakeStop)
    {
        m_CoralIntakeMotor.Set(0.00);
    }

    if(m_coralAngleState == AngleUp)
    {
        m_CoralAnglePid.SetSetpoint(Constants::kCoralAngleUp);
        double angleMotorValue = m_CoralAnglePid.Calculate(m_CoralAngleEncoder.Get());
        angleMotorValue = std::clamp(angleMotorValue, -Constants::kCoralAngleSpeed, Constants::kCoralAngleSpeed );
        m_CoralAngleMotor.Set(angleMotorValue);// motor is reversed
    }
    else if(m_coralAngleState == AngleDn)
    {
        m_CoralAnglePid.SetSetpoint(Constants::kCoralAngleDn);
        double angleMotorValue = m_CoralAnglePid.Calculate(m_CoralAngleEncoder.Get());
        angleMotorValue = std::clamp(angleMotorValue, -Constants::kCoralAngleSpeed, Constants::kCoralAngleSpeed );
        m_CoralAngleMotor.Set(angleMotorValue);
    }
    else if(m_coralAngleState == AngleStop)
    {
        m_CoralAngleMotor.Set(0.00);
    }
    
}


// ****************************************************************************
void Claw::ChangeState( CoralAngleState_t Astate, CoralIntakeState_t Istate )
{
    m_coralAngleState = Astate;
    m_coralIntakeState = Istate;
}

// ****************************************************************************
void Claw::UpdateSmartDashboardData( )
{
    frc::SmartDashboard::PutNumber("Coral Angle Encoder Value", m_CoralAngleEncoder.Get());
    frc::SmartDashboard::PutNumber("Coral Angle Setpoint", m_CoralAnglePid.GetSetpoint());
    frc::SmartDashboard::PutNumber("Coral Intake Motor Encoder Value", m_CoralIntakeEncoder.GetPosition());
    frc::SmartDashboard::PutNumber("Coral Angle motor value", m_CoralAngleMotor.Get());
}
