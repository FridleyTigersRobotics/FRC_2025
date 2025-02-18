#include <Debug.h>
#include <Climber.h>
#include <frc/smartdashboard/SmartDashboard.h>



// ****************************************************************************
void Climber::Init()
{
   winch_calibrated=false;
}


// ****************************************************************************
void Climber::TeleopInit()
{
   winch_calibrated=false;
   m_ClimberState = ClimberStop;
}


// ****************************************************************************
void Climber::ManualControl( double climbSpeed )
{
    m_ClimbSpeed = climbSpeed;
    m_ClimberState = ClimberStop;
}


// ****************************************************************************
void Climber::Update()
{
     if ( m_ManualClimbControl )
    {
        m_ClimbMotorEast.Set( m_ClimbSpeed );
        m_ClimbMotorWest.Set( -m_ClimbSpeed );
    }

    if ( winch_limit.Get() )
    {
        m_climberEncoderEast.Reset();
        m_climberEncoderWest.Reset();
        winch_calibrated = true;
    }

    if (m_ClimberState == ClimberReset)
    {
        if(winch_calibrated==false)
        {
            m_ClimbMotorEast.Set( Constants::kClimbCalibrateSpeed ); // //winch in (climber down, robot climb up)
            m_ClimbMotorWest.Set( -Constants::kClimbCalibrateSpeed );  
        }
        else if( winch_calibrated==true && ( m_climberEncoderWest.Get() < maxClimbWest ) )
        {
            m_ClimbMotorEast.Set( -Constants::kClimbCalibrateSpeed);
            m_ClimbMotorWest.Set( Constants::kClimbCalibrateSpeed );  
        }
        else if( winch_calibrated==true && ( m_climberEncoderWest.Get() >= maxClimbWest ) )
        {
            m_ClimbMotorEast.Set( 0.0 );
            m_ClimbMotorWest.Set( 0.0 );
            m_ClimberState = ClimberStop;
        }
    }
    
    if (m_ClimberState == ClimberStop)
    {
        m_ClimbMotorEast.Set( 0.0 );
        m_ClimbMotorWest.Set( 0.0 );
    }
        
}



// ****************************************************************************
void Climber::ChangeState( ClimberState_t state )
{
   m_ClimberState = state;
}


// ****************************************************************************
void Climber::UpdateSmartDashboardData( )
{
    frc::SmartDashboard::PutBoolean("Winch Limit", winch_limit.Get());
    frc::SmartDashboard::PutNumber("Climb Encoder West", m_climberEncoderWest.Get());
    frc::SmartDashboard::PutNumber("Climb Encoder East", m_climberEncoderEast.Get());
   
}
