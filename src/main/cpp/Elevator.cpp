#include <Debug.h>
#include <Elevator.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <rev/SparkMax.h>
#include <rev/config/SparkMaxConfig.h>
using namespace rev::spark;

bool elevatormoving = false;

bool Elevator::ismoving()
{
    return elevatormoving;
}


// ****************************************************************************
void Elevator::Init()
{
    m_ElevatorState = ElevatorStop;
    m_Elevator0Encoder.SetPosition( 0.0 );
    m_Elevator1Encoder.SetPosition( 0.0 );
    m_ElevatorPid.SetTolerance(0.001);
    m_ElevatorPid.Reset();
    m_Motor1.SetInverted(true);
}

// ****************************************************************************
void Elevator::TeleopInit()
{
    m_ElevatorState = ElevatorStop;
}


// ****************************************************************************
void Elevator::Update()
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
    if(fabs(m_Motor0.Get())>0.1 || fabs(m_Motor1.Get())>0.1)
    {
        elevatormoving = true;
    }
    else if(fabs(m_Motor0.Get())<=0.1 || fabs(m_Motor1.Get())<=0.1)
    {
       elevatormoving = false;
    }
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
    frc::SmartDashboard::PutNumber("Elevator Encoder Value", m_Elevator0Encoder.GetPosition());
    frc::SmartDashboard::PutNumber("Elevator  Setpoint", m_ElevatorPid.GetSetpoint());
    frc::SmartDashboard::PutNumber("Elevator Motor Value", m_Motor0.Get());
    frc::SmartDashboard::PutBoolean("Elevator Moving", elevatormoving);

}


// ****************************************************************************
void Elevator::ManualControl()
{
 
}

