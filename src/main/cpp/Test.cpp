#include <Robot.h>
#include <frc/MathUtil.h>
#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <fmt/printf.h>
#include <frc/filter/SlewRateLimiter.h>

#include <Drivetrain.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <string>
#include <Climber.h>
#include <networktables/NetworkTable.h>
#include <LimelightHelpers.h>
#include <cameraserver/CameraServer.h>

void Robot::TestInit()
{
    m_Drivetrain.m_imu.ZeroYaw();
     m_fieldRelative = true;
     
    m_Climber.TeleopInit();
    m_Claw.TeleopInit();
}


// ****************************************************************************
void Robot::TestPeriodic() 
{ 
  m_Drivetrain.UpdateOdometry();
  double DriveDeadband = 0.1;
  double DriveX = frc::ApplyDeadband( m_driveController.GetLeftY(), DriveDeadband );
  double DriveY = frc::ApplyDeadband( m_driveController.GetLeftX(), DriveDeadband );
  double driveRotSpeed = frc::ApplyDeadband( m_driveController.GetRightX(), DriveDeadband );
  const auto rotationSpeed = driveRotSpeed * Drivetrain::kMaxAngularSpeed;
  const auto xSpeed = DriveX * Drivetrain::kMaxSpeed;
  const auto ySpeed = DriveY * Drivetrain::kMaxSpeed;

  frc::SmartDashboard::PutNumber( "xSpeed", double{xSpeed} );
  frc::SmartDashboard::PutNumber( "ySpeed", double{ySpeed} );

  m_Drivetrain.SetSpeeds( xSpeed, ySpeed, rotationSpeed );


  // if ( m_driveController.GetAButton() )
  // {
  //   m_Claw.ChangeState(  m_Claw.CoralClawUp );
  // }
  // else if ( m_driveController.GetBButton() )
  // {
  //   m_Claw.ChangeState(  m_Claw.CoralClawDown );
  // }
  // else
  // {
  //   m_Claw.ChangeState( m_Claw.CoralClawStop );
  // }

  // if ( m_driveController.GetAButton() )
  // {
  //   m_Claw.ManualControl( 0.0, 0.5 );
  // }
  // else if ( m_driveController.GetBButton() )
  // {
  //   m_Claw.ManualControl( 0.0, -0.5 );
  // }
  // else
  // {
  //   m_Claw.ManualControl( 0.0, 0.0 );
  // }


  if ( m_buttons.GetRawButton(1) )
  {
    m_Elevator.ChangeState( m_Elevator.ElevatorCoralL4 );
  }




  
  if ( m_driveController.GetBButtonPressed() )
  {
    m_Elevator.ChangeState( m_Elevator.ElevatorStartingConfig );
  }
  


#if 0
  if(m_buttons.GetRawButton(1))//winch in
  {
    m_Climber.ChangeState( m_Climber.ClimberWinchInManual, m_Climber.GrabMaintain );
  }
  else if(m_buttons.GetRawButton(4))//winch out
  {
    m_Climber.ChangeState( m_Climber.ClimberWinchOutManual, m_Climber.GrabMaintain );
  }
  else
  {
    m_Climber.ChangeState( m_Climber.ClimberWinchStop, m_Climber.GrabMaintain );
  }

  if(m_buttons.GetRawButton(2))//grab
  {
    m_Climber.ChangeState( m_Climber.ClimberWinchMaintain, m_Climber.GrabHorizontal );
  }
  else if(m_buttons.GetRawButton(5))//release
  {
    m_Climber.ChangeState( m_Climber.ClimberWinchMaintain, m_Climber.GrabVertical );
  }
  else
  {
    m_Climber.ChangeState( m_Climber.ClimberWinchMaintain, m_Climber.GrabStop); // do want to stop grabber state here
  }
#endif

  // Update all subsystems
  m_Drivetrain.Update( GetPeriod(), m_fieldRelative );
  m_Climber.Update();
  m_Elevator.Update();
  m_Claw.Update( m_Elevator.ismoving() );
}