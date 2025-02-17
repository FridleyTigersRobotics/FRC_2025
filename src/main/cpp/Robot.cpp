// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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



// ****************************************************************************
void Robot::RobotInit()
{

  m_Drivetrain.m_imu.Reset();
  //frc::CameraServer::StartAutomaticCapture();
  // Autonomous Chooser
  m_autoChooser.SetDefaultOption( kAutoNameDefault,         kAutoNameDefault );
  m_autoChooser.AddOption       ( kAutoDrive,               kAutoDrive );
  frc::SmartDashboard::PutNumber("AutoModeInt", 0 );
  frc::SmartDashboard::PutData("Auto Modes", &m_autoChooser);

  // Initialize Subsystems
  m_Drivetrain.Init();
  m_Climber.Init();
  m_Elevator.Init();
  m_Claw.Init();

}


// ****************************************************************************
void Robot::RobotPeriodic()
{
  m_Drivetrain.UpdateSmartDashboardData();
  m_Climber.UpdateSmartDashboardData();
  m_Elevator.UpdateSmartDashboardData();
  m_Claw.UpdateSmartDashboardData();
}


// ****************************************************************************
void Robot::DisabledInit()
{

}


// ****************************************************************************
void Robot::TeleopInit() 
{
  m_Drivetrain.m_imu.ZeroYaw();
  m_DriveRotatePid.EnableContinuousInput(-180, 180);

  m_fieldRelative = true;
  Angle=0;

  m_AutoXdirPid.SetTolerance( kXyPosTolerance,  kXyVelTolerance );
  m_AutoXdirPid.Reset( 0.0_m );


}


// ****************************************************************************
void Robot::TeleopPeriodic() 
{ 
  frc::SmartDashboard::PutNumber( "GetYaw", double{m_Drivetrain.GetYaw()} );
  frc::SmartDashboard::PutNumber( "m_Drivetrain.m_imu.GetYaw()", double{m_Drivetrain.m_imu.GetYaw()} );
  frc::SmartDashboard::PutNumber( "m_Drivetrain.m_imu.GetAngle()", double{m_Drivetrain.m_imu.GetAngle()} );
  m_Drivetrain.UpdateOdometry();
  double DriveDeadband = 0.1;
  double DriveX = frc::ApplyDeadband( m_driveController.GetLeftY(), DriveDeadband );
  double DriveY = frc::ApplyDeadband( m_driveController.GetLeftX(), DriveDeadband );


#if 0

  double y     = m_driveController.GetRightY();
  double x     = m_driveController.GetRightX();
  double mag   = sqrt( x * x + y * y );
  double angle = ((atan2( x , y ) * 180 / std::numbers::pi)+180);

  frc::SmartDashboard::PutNumber( "InputAngle", double{angle} );
  frc::SmartDashboard::PutNumber( "InputMag", double{mag} );
  if( mag > 0.8 )
  {
    m_DriveTargetAngle = angle;
    m_DriveRotatePid.SetSetpoint( m_DriveTargetAngle );
  }


  double driveRotSpeedUnClamped = m_DriveRotatePid.Calculate( m_Drivetrain.GetAngle() );
  double driveRotSpeed          = std::clamp( driveRotSpeedUnClamped, -1.0, 1.0 );
  // Get the rate of angular rotation. We are inverting this.
  const auto rotationSpeedUnClamped = ( m_DriveRotatePid.GetError() > 5.0 ) ? ( driveRotSpeed * ( units::radians_per_second_t{0.0} ) ) : ( units::radians_per_second_t{0.0} );
  const auto rotationSpeed =  driveRotSpeedUnClamped * Drivetrain::kMaxAngularSpeed;

  frc::SmartDashboard::PutNumber( "Err.", m_DriveRotatePid.GetError());
  frc::SmartDashboard::PutNumber( "controler_mag",   mag );
  frc::SmartDashboard::PutNumber( "controler_angle", angle );

  if( mag > 0.9 )
  {
    targetWrappedAngle = angle;
    angleChanged       = true;
  }

  if ( angleChanged )
  {
    double unwrappedRobotAngle = m_Drivetrain.GetAngle();
    double wrappedRobotAngle   = m_Drivetrain.GetYaw();
    double angleDelta = targetWrappedAngle - wrappedRobotAngle;
    if ( angleDelta > 180 )
    {
      angleDelta -= 360;
    }
    if ( angleDelta <= -180 )
    {
      angleDelta += 360;
    }

    m_DriveTargetAngle = unwrappedRobotAngle + angleDelta;
  frc::SmartDashboard::PutNumber( "HEADING_unwrappedRobotAngle",       unwrappedRobotAngle);
  frc::SmartDashboard::PutNumber( "HEADING_angleDelta",       angleDelta);











#else
  double driveRotSpeed = frc::ApplyDeadband( m_driveController.GetRightX(), DriveDeadband );
  const auto rotationSpeed = driveRotSpeed * Drivetrain::kMaxAngularSpeed;
#endif


  // Get the x speed. We are inverting this because Xbox controllers return
  // negative values when we push forward.
  const auto xSpeed = DriveX * Drivetrain::kMaxSpeed;

  // Get the y speed or sideways/strafe speed. We are inverting this because
  // we want a positive value when we pull to the left. Xbox controllers
  // return positive values when you pull to the right by default.
  const auto ySpeed = DriveY * Drivetrain::kMaxSpeed;
//Eli's dumbass spaghetti code for field oriented
/*if (Angle != 0 && Angle != m_Drivetrain.Drivetrain::m_imu.GetAngle())
{
  DumbAssOffset + (Angle-m_Drivetrain.Drivetrain::m_imu.GetAngle());
  Angle = m_Drivetrain.Drivetrain::m_imu.GetAngle();
}
else
{
  Angle = m_Drivetrain.Drivetrain::m_imu.GetAngle();
}*/

/*

Set Angle to Angle+(How much we're turning times -1)
-AngleCrap is the thing from the auto chooser.
Rotate wheels to where they're supposed to be offset by Angle

*/
  frc::SmartDashboard::PutNumber( "xSpeed", double{xSpeed} );
  frc::SmartDashboard::PutNumber( "ySpeed", double{ySpeed} );

  m_Drivetrain.SetSpeeds( xSpeed, ySpeed, rotationSpeed );


  if ( m_driveController.GetAButton() )
  {
    m_Claw.ChangeState(  m_Claw.CoralClawUp );
  }
  else if ( m_driveController.GetBButton() )
  {
    m_Claw.ChangeState(  m_Claw.CoralClawDown );
  }
  else
  {
    m_Claw.ChangeState( m_Claw.CoralClawStop );
  }

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


  
  // Update all subsystems
  m_Drivetrain.Update( GetPeriod(), m_fieldRelative );
  m_Climber.Update();
  m_Elevator.Update();
  m_Claw.Update();
}



#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
