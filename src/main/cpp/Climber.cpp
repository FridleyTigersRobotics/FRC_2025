#include <Debug.h>
#include <Climber.h>
#include <frc/smartdashboard/SmartDashboard.h>



// ****************************************************************************
void Climber::Init()
{
   winch_calibrated=false;
   m_ClimberState = ClimberStop;
   m_CageEncoder.SetPosition(0.0);
}


// ****************************************************************************
void Climber::TeleopInit()
{
   winch_calibrated=false;
   m_ClimberState = ClimberStop;
   m_CageEncoder.SetPosition(0.0);
   m_GrabberPid.SetIntegratorRange( -0.1, 0.1 ); //stops integrator wind-up
   m_GrabberPid.SetTolerance(1.0);
   m_GrabberPid.Reset();
}


// ****************************************************************************
void Climber::ManualControl( double climbSpeed , double grabSpeed )
{
    m_ClimbSpeed = climbSpeed;
    m_GrabSpeed = grabSpeed;
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

    if(m_ClimberState == GrabSpin)
    {
        m_GrabberPid.SetSetpoint(Constants::kGrab90);
        double grabMotorValue = m_GrabberPid.Calculate(m_CageEncoder.GetPosition());
        grabMotorValue = std::clamp(grabMotorValue, -0.5, 0.5 );
        m_CageGrabberMotor.Set(grabMotorValue);
    }
    
    if (m_ClimberState == ClimberStop)
    {
        m_ClimbMotorEast.Set( 0.0 );
        m_ClimbMotorWest.Set( 0.0 );

        m_GrabberPid.SetSetpoint(0.00);
        double grabMotorValue = m_GrabberPid.Calculate(m_CageEncoder.GetPosition());
        grabMotorValue = std::clamp(grabMotorValue, -Constants::kGrabSpeed, Constants::kGrabSpeed );
        m_CageGrabberMotor.Set(grabMotorValue);
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
    frc::SmartDashboard::PutNumber("Climb Winch Encoder West", m_climberEncoderWest.Get());
    frc::SmartDashboard::PutNumber("Climb Winch Encoder East", m_climberEncoderEast.Get());
    frc::SmartDashboard::PutNumber("Climber: Cage Grabber", m_CageEncoder.GetPosition());
   
}
