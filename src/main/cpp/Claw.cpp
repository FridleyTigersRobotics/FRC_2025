#include <Debug.h>
#include <Claw.h>
#include <frc/smartdashboard/SmartDashboard.h>



// ****************************************************************************
void Claw::Init()
{
   
}


// ****************************************************************************
void Claw::ManualControl()
{
    
}


// ****************************************************************************
void Claw::Update()
{
   
}


// ****************************************************************************
void Claw::ChangeState( ClawState_t ClawState )
{
 
}


// ****************************************************************************
void Claw::UpdateSmartDashboardData( )
{
    frc::SmartDashboard::PutNumber("Intake: Algae Encoder Value", m_AlgaeEncoder.Get());
    frc::SmartDashboard::PutNumber("Intake: Coral Encoder Value", m_CoralEncoder.Get());
}
