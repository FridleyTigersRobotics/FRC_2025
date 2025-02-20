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

  m_fieldRelative = true;

  m_AutoXdirPid.SetTolerance( kXyPosTolerance,  kXyVelTolerance );
  m_AutoXdirPid.Reset( 0.0_m );
  m_Climber.TeleopInit();
  m_Claw.TeleopInit();

}


// ****************************************************************************
void Robot::TeleopPeriodic() 
{ 
  m_Drivetrain.UpdateOdometry();

  double DriveDeadband = 0.1;
  double DriveX = frc::ApplyDeadband( m_driveController.GetLeftX(), DriveDeadband );
  double DriveY = frc::ApplyDeadband( m_driveController.GetLeftY(), DriveDeadband );
  double RotX   = frc::ApplyDeadband( m_driveController.GetRightX(), DriveDeadband );
  double RotY   = frc::ApplyDeadband( m_driveController.GetRightY(), DriveDeadband );

  m_Drivetrain.Drive( DriveX, DriveY, RotX, RotY, m_fieldRelative );

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
    m_Climber.ChangeState( m_Climber.ClimberWinchMaintain, m_Climber.GrabMaintain); // don't want to stop grabber state here
  }

  if(m_buttons.GetRawButton(7))//calibrate climber
  {
    m_Climber.ChangeState( m_Climber.ClimberWinchCalibrate, m_Climber.GrabMaintain );
  }


  // Update all subsystems
  m_Drivetrain.Update( GetPeriod() );
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
