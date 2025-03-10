// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Climber.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/SparkMax.h>
#include <rev/config/SparkMaxConfig.h>
#include <frc/shuffleboard/Shuffleboard.h>

int climberholdpos = 0;
bool grabberhoriz = false;

// ****************************************************************************
Climber::Climber()
{
   winch_calibrated=false;
   m_ClimberWinchState = ClimberWinchStop;
   m_ClimberGrabberState = GrabStop;
   m_CageEncoder.SetPosition(0.0);
   m_GrabberPid.SetIntegratorRange( -0.1, 0.1 ); //stops integrator wind-up
   m_GrabberPid.SetTolerance(Constants::kGrabPidTol);
   m_GrabberPid.Reset();

    rev::spark::SparkMaxConfig GrabMotorConfig;
    GrabMotorConfig
        .Inverted(false)
        .SetIdleMode(rev::spark::SparkMaxConfig::IdleMode::kBrake)
        .VoltageCompensation(12.0)
        .SmartCurrentLimit(15);

    //m_AlgaeAngleMotor.SetInverted(true); this is depreciated
    m_CageGrabberMotor.Configure(GrabMotorConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);

    m_ClimbMotorEast.SetNeutralMode(NeutralMode::Brake);
    m_ClimbMotorWest.SetNeutralMode(NeutralMode::Brake);

    grabtimer.Reset();
    grabtimer.Start();

    m_ClimbMotorEast.Set( 0.0 );
    m_ClimbMotorWest.Set( 0.0 );
    m_ClimbPid.SetIntegratorRange(-0.1,0.1);//stops integrator wind-up
    m_ClimbPid.SetTolerance( Constants::kClimbPidTol );
    m_ClimbPid.Reset();
    climberholdpos = 0.00;

};


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
void Climber::Periodic()
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
            #if 0 // if using timer to determine when to start climb
                auto elapsed_ask_to_grab = grabtimer.Get();
                if(elapsed_ask_to_grab > Constants::kGrabDelay)
                {
                    m_ClimbMotorEast.Set( Constants::kClimbSpeed ); //winch in (climber down, robot climb up)
                    m_ClimbMotorWest.Set( -Constants::kClimbSpeed );
                }
                else
                {
                    m_ClimbMotorEast.Set( 0.00 ); //waiting for grabtimer time
                    m_ClimbMotorWest.Set( 0.00 );
                }
            #endif
            #if 1  //if using position to determine when to start climb
                if(fabs(m_GrabberPid.GetError())<Constants::kWinchTolerance)
                {
                    grabberhoriz = true;
                    m_ClimbMotorEast.Set( Constants::kClimbSpeed ); //winch in (climber down, robot climb up)
                    m_ClimbMotorWest.Set( -Constants::kClimbSpeed );
                }
                else
                {
                    m_ClimbMotorEast.Set( 0.00 ); //waiting for grabtimer time
                    m_ClimbMotorWest.Set( 0.00 );
                }
            #endif
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

    if(m_ClimberWinchState == ClimberWinchHold)
    {
        if(winch_limit.Get())
        {
            m_ClimbMotorEast.Set( 0.0 );
            m_ClimbMotorWest.Set( 0.0 );
        }
        else
        {
            double winchspeed=0.00;
            m_ClimbPid.SetSetpoint(climberholdpos-Constants::kClimbSetpointOffset);
            winchspeed = m_ClimbPid.Calculate(m_climberEncoderWest.Get());
            winchspeed = std::clamp(winchspeed, -Constants::kClimbSpeed, Constants::kClimbSpeed);
            m_ClimbMotorEast.Set( -winchspeed);
            m_ClimbMotorWest.Set( winchspeed );  
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
        grabtimer.Restart();
        grabberhoriz = false;
    }

    if(m_ClimberGrabberState == GrabStop)
    {
        m_CageGrabberMotor.Set(0.00);
    }
}



// ****************************************************************************
frc2::CommandPtr Climber::ChangeStateCommand( ClimberWinchState_t Cstate, ClimberGrabberState_t Gstate )
{
    return RunOnce([ this, Cstate, Gstate ] { ChangeState( Cstate, Gstate ); });
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
        climberholdpos = m_climberEncoderWest.Get();
        m_ClimberGrabberState = Gstate;
   }

}









// ****************************************************************************
void Climber::UpdateSmartDashboardData( )
{
    //frc::SmartDashboard::PutBoolean("Winch Limit Switch", winch_limit.Get());
    //frc::SmartDashboard::PutNumber("Climb Winch Encoder West", m_climberEncoderWest.Get());
    //frc::SmartDashboard::PutNumber("Climb Winch Encoder East", m_climberEncoderEast.Get());
    //frc::SmartDashboard::PutNumber("Climber: Cage Grabber", m_CageEncoder.GetPosition());
    //frc::SmartDashboard::PutNumber("grabber status", m_ClimberGrabberState);
    //frc::SmartDashboard::PutNumber("climber PID setpoint", m_ClimbPid.GetSetpoint());
    m_grabhoriz->SetBoolean(grabberhoriz);

}


