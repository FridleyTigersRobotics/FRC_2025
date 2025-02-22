#include <Debug.h>
#include <Climber.h>
#include <frc/smartdashboard/SmartDashboard.h>



// ****************************************************************************
void Climber::Init()
{
   winch_calibrated=false;
   m_ClimberWinchState = ClimberWinchStop;
   m_ClimberGrabberState = GrabStop;
   m_CageEncoder.SetPosition(0.0);
   m_GrabberPid.SetIntegratorRange( -0.1, 0.1 ); //stops integrator wind-up
   m_GrabberPid.SetTolerance(1.0);
   m_GrabberPid.Reset();
}


// ****************************************************************************
void Climber::TeleopInit()
{
   winch_calibrated=false;
   m_ClimberWinchState = ClimberWinchStop;
   m_ClimberGrabberState = GrabVertical;
}


// ****************************************************************************
void Climber::ManualControl()
{

}


// ****************************************************************************
void Climber::Update()
{
    if ( winch_limit.Get() )
    {
        m_climberEncoderEast.Reset();
        m_climberEncoderWest.Reset();
        winch_calibrated = true;
    }
    
    if(m_ClimberWinchState == ClimberWinchInManual)
    {
        if(!winch_limit.Get())
        {
            m_ClimbMotorEast.Set( Constants::kClimbSpeed ); //winch in (climber down, robot climb up)
            m_ClimbMotorWest.Set( -Constants::kClimbSpeed );
        }
        else
        {
            m_ClimbMotorEast.Set( 0.00 ); //winch in (climber down, robot climb up)
            m_ClimbMotorWest.Set( 0.00 );
        }
       
    }

    if(m_ClimberWinchState == ClimberWinchOutManual)
    {
        if(winch_calibrated == false)
        {
             m_ClimbMotorEast.Set( -Constants::kClimbSpeed ); //winch out (climber up)
             m_ClimbMotorWest.Set( Constants::kClimbSpeed );  
        }
        else if(winch_calibrated == true)
        {
            if( m_climberEncoderWest.Get() < maxClimbWest )
            {
                m_ClimbMotorEast.Set( -Constants::kClimbCalibrateSpeed);
                m_ClimbMotorWest.Set( Constants::kClimbCalibrateSpeed );  
            }
            else if( m_climberEncoderWest.Get() >= maxClimbWest )
            {
                m_ClimbMotorEast.Set( 0.0 );
                m_ClimbMotorWest.Set( 0.0 );
            }
        }
        
    }

    //if(m_ClimberWinchState == ClimberWinchMaintain)
    //{
    //    //do not change state, ignore update in Climber::ChangeState
    //}

    if (m_ClimberWinchState == ClimberWinchCalibrate)
    {
        if(winch_calibrated==false)
        {
            m_ClimbMotorEast.Set( Constants::kClimbCalibrateSpeed ); //winch in (climber down, robot climb up)
            m_ClimbMotorWest.Set( -Constants::kClimbCalibrateSpeed );  
        }
        else if( winch_calibrated==true && ( m_climberEncoderWest.Get() < maxClimbWest/2 ) )
        {
            m_ClimbMotorEast.Set( -Constants::kClimbCalibrateSpeed);
            m_ClimbMotorWest.Set( Constants::kClimbCalibrateSpeed );  
        }
        else if( winch_calibrated==true && ( m_climberEncoderWest.Get() >= maxClimbWest/2 ) )
        {
            m_ClimbMotorEast.Set( 0.0 );
            m_ClimbMotorWest.Set( 0.0 );
            m_ClimberWinchState = ClimberWinchStop;
        }
    }
    
    if (m_ClimberWinchState == ClimberWinchStop)
    {
        m_ClimbMotorEast.Set( 0.0 );
        m_ClimbMotorWest.Set( 0.0 ); 
    }
    
    if(m_ClimberGrabberState == GrabHorizontal)
    {
        m_GrabberPid.SetSetpoint(Constants::kGrab90);
        double grabMotorValue = m_GrabberPid.Calculate(m_CageEncoder.GetPosition());
        grabMotorValue = std::clamp(grabMotorValue, -Constants::kGrabSpeed, Constants::kGrabSpeed );
        m_CageGrabberMotor.Set(grabMotorValue);
    }

    if(m_ClimberGrabberState == GrabVertical)
    {
        m_GrabberPid.SetSetpoint(0.00);
        double grabMotorValue = m_GrabberPid.Calculate(m_CageEncoder.GetPosition());
        grabMotorValue = std::clamp(grabMotorValue, -Constants::kGrabSpeed, Constants::kGrabSpeed );
        m_CageGrabberMotor.Set(grabMotorValue);
    }

    if(m_ClimberGrabberState == GrabStop)
    {
        m_CageGrabberMotor.Set(0.00);
    }
}



// ****************************************************************************
void Climber::ChangeState( ClimberWinchState_t Cstate, ClimberGrabberState_t Gstate )
{
   if(Cstate != ClimberWinchMaintain)
   {
    m_ClimberWinchState = Cstate;
   }

   if(Gstate != GrabMaintain)
   {
    m_ClimberGrabberState = Gstate;
   }

}


// ****************************************************************************
void Climber::UpdateSmartDashboardData( )
{
    frc::SmartDashboard::PutBoolean("Winch Limit Switch", winch_limit.Get());
    frc::SmartDashboard::PutNumber("Climb Winch Encoder West", m_climberEncoderWest.Get());
    frc::SmartDashboard::PutNumber("Climb Winch Encoder East", m_climberEncoderEast.Get());
    frc::SmartDashboard::PutNumber("Climber: Cage Grabber", m_CageEncoder.GetPosition());
    frc::SmartDashboard::PutNumber("grabber staus", m_ClimberGrabberState);
}
