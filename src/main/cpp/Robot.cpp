// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
/*
~~~~~Steps 2 field orientation~~~~~~
1)Reset
Starting offset is set from auto
Current=0, AngleChanged=0

2) Variables
void awake()
{
Current angle Starting offset
}
- Angle changed
  +-|Current angle-Target angle|

3)when drive
- the when the wheel-AngleChanged???
*/
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
  Angle=0;
  m_Drivetrain.m_imu.Reset();
  frc::CameraServer::StartAutomaticCapture();
  // Autonomous Chooser
  m_autoChooser.SetDefaultOption( kAutoNameDefault,         kAutoNameDefault );
  m_autoChooser.AddOption       ( kAutoDrive,               kAutoDrive );
  frc::SmartDashboard::PutNumber("AutoModeInt", 0 );
  frc::SmartDashboard::PutData("Auto Modes", &m_autoChooser);
}


// ****************************************************************************
void Robot::RobotPeriodic()
{

}


// ****************************************************************************
void Robot::DisabledInit()
{

}


// ****************************************************************************
void Robot::TeleopInit() 
{

  m_DriveRotatePid.EnableContinuousInput(-180, 180);

  m_fieldRelative = true;



  m_AutoXdirPid.SetTolerance( kXyPosTolerance,  kXyVelTolerance );
  m_AutoXdirPid.Reset( 0.0_m );

  // Initialize Subsystems
  m_Drivetrain.Init();
  m_Climber.Init();
  m_Elevator.Init();
  m_Claw.Init();
}


// ****************************************************************************
void Robot::TeleopPeriodic() 
{ 
  frc::SmartDashboard::PutNumber( "GetYaw", double{m_Drivetrain.GetYaw()} );
  frc::SmartDashboard::PutNumber( "m_Drivetrain.m_imu.GetYaw()", double{m_Drivetrain.m_imu.GetYaw()} );
  frc::SmartDashboard::PutNumber( "m_Drivetrain.m_imu.GetAngle()", double{m_Drivetrain.m_imu.GetAngle()} );

  double DriveDeadband = 0.1;
  double DriveX = frc::ApplyDeadband( m_driveController.GetLeftY(), DriveDeadband );
  double DriveY = frc::ApplyDeadband( m_driveController.GetLeftX(), DriveDeadband );

#if 0
  double y     = m_driveController.GetRightY();
  double x     = m_driveController.GetRightX();
  double mag   = sqrt( x * x + y * y );
  double angle = ((atan2( x , y ) * 180 / std::numbers::pi) +180) * -1;

  frc::SmartDashboard::PutNumber( "InputAngle", double{angle} );

  if( mag > 0.9 )
  {
    m_DriveTargetAngle = angle;
    m_DriveRotatePid.SetSetpoint( m_DriveTargetAngle );
  }


  double driveRotSpeedUnClamped = m_DriveRotatePid.Calculate( m_Drivetrain.GetYaw() );
  double driveRotSpeed          = std::clamp( driveRotSpeedUnClamped, -0.5, 0.5 );
  // Get the rate of angular rotation. We are inverting this.
  const auto rotationSpeed = ( m_DriveRotatePid.GetError() > 3.0 ) ? ( driveRotSpeed * Drivetrain::kMaxAngularSpeed ) : ( units::radians_per_second_t{0.0} );

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
  Angle=m_Drivetrain.Drivetrain::GetYaw();
/*

Set Angle to Angle+(How much we're turning times -1)
-AngleCrap is the thing from the auto chooser.
Rotate wheels to where they're supposed to be offset by Angle

*/
  frc::SmartDashboard::PutNumber( "xSpeed", double{xSpeed} );
  frc::SmartDashboard::PutNumber( "ySpeed", double{ySpeed} );

  m_Drivetrain.SetSpeeds( xSpeed, ySpeed, rotationSpeed );

  
  // Update all subsystems
  m_Drivetrain.Update( GetPeriod(), m_fieldRelative );
  m_Climber.Update();
  m_Elevator.Update();
  m_Claw.Update();
}



#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
