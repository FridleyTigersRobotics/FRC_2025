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
  m_Elevator.TeleopInit();

}


// ****************************************************************************
void Robot::TeleopPeriodic() 
{ 
  m_Drivetrain.UpdateOdometry();

  double DriveDeadband = 0.1;
  double DriveX = frc::ApplyDeadband( m_driveController.GetLeftY(), DriveDeadband );
  double DriveY = frc::ApplyDeadband( m_driveController.GetLeftX(), DriveDeadband );
  double driveRotSpeed = frc::ApplyDeadband( m_driveController.GetRightX(), DriveDeadband );
  const auto rotationSpeed = driveRotSpeed * Drivetrain::kMaxAngularSpeed;



  // Get the x speed. We are inverting this because Xbox controllers return
  // negative values when we push forward.
  const auto xSpeed = DriveX * Drivetrain::kMaxSpeed;

  // Get the y speed or sideways/strafe speed. We are inverting this because
  // we want a positive value when we pull to the left. Xbox controllers
  // return positive values when you pull to the right by default.
  const auto ySpeed = DriveY * Drivetrain::kMaxSpeed;


  frc::SmartDashboard::PutNumber( "xSpeed", double{xSpeed} );
  frc::SmartDashboard::PutNumber( "ySpeed", double{ySpeed} );
  m_Drivetrain.SetSpeeds( xSpeed, ySpeed, rotationSpeed );

// ****************************************************************************
 if(m_buttons.GetRawButton(10))//all down
  {
    m_Claw.ChangeState( m_Claw.AngleUp, m_Claw.intakeStop );
    m_Elevator.ChangeState( m_Elevator.ElevatorStartingConfig );
  }
  if(m_buttons.GetRawButton(5))//Coral Intake
  {
    m_Claw.ChangeState( m_Claw.AngleDn, m_Claw.intakeStop );
    m_Elevator.ChangeState( m_Elevator.ElevatorCoralIntake );
  }
  if(m_buttons.GetRawButton(1))//Coral L1
  {
    m_Claw.ChangeState( m_Claw.AnglePlaceCoral, m_Claw.intakeStop );
    m_Elevator.ChangeState( m_Elevator.ElevatorCoralL1 );
  }
  if(m_buttons.GetRawButton(2))//Coral L2
  {
    m_Claw.ChangeState( m_Claw.AnglePlaceCoral, m_Claw.intakeStop );
    m_Elevator.ChangeState( m_Elevator.ElevatorCoralL2 );
  }
  if(m_buttons.GetRawButton(3))//Coral L3
  {
    m_Claw.ChangeState( m_Claw.AnglePlaceCoral, m_Claw.intakeStop );
    m_Elevator.ChangeState( m_Elevator.ElevatorCoralL3 );
  }
  if(m_buttons.GetRawButton(4))//Coral L4
  {
    m_Claw.ChangeState( m_Claw.AnglePlaceCoral, m_Claw.intakeStop );
    m_Elevator.ChangeState( m_Elevator.ElevatorCoralL4 );
  }
  if(m_buttons.GetRawButton(7))//Coral Intake
  {
    m_Claw.ChangeState( m_Claw.AngleMaintain, m_Claw.intakeIntake );
    m_Elevator.ChangeState( m_Elevator.ElevatorMaintain);
  }
  if(m_buttons.GetRawButton(8))//Coral reverse
  {
    m_Claw.ChangeState( m_Claw.AngleMaintain, m_Claw.intakeReverse );
    m_Elevator.ChangeState( m_Elevator.ElevatorMaintain );
  }
  if(!m_buttons.GetRawButton(8) && !m_buttons.GetRawButton(7))//Coral Intake stop
  {
    m_Claw.ChangeState( m_Claw.AngleMaintain, m_Claw.intakeStop );
    m_Elevator.ChangeState( m_Elevator.ElevatorMaintain);
  }
  if(m_buttons.GetRawButton(5))//climber out
  {
    m_Claw.ChangeState( m_Claw.AngleUp, m_Claw.intakeStop );
    m_Elevator.ChangeState( m_Elevator.ElevatorStartingConfig );
    m_Climber.ChangeState( m_Climber.ClimberWinchOutManual, m_Climber.GrabVertical );

  }
  if(m_buttons.GetRawButton(6))//climber in
  {
    m_Claw.ChangeState( m_Claw.AngleUp, m_Claw.intakeStop );
    m_Elevator.ChangeState( m_Elevator.ElevatorStartingConfig );
    m_Climber.ChangeState( m_Climber.ClimberWinchInManual, m_Climber.GrabHorizontal );

  }
  if(!m_buttons.GetRawButton(5) && !m_buttons.GetRawButton(6))//climber stop
  {
    m_Claw.ChangeState( m_Claw.AngleMaintain, m_Claw.intakeMaintain );
    m_Elevator.ChangeState( m_Elevator.ElevatorMaintain );
    m_Climber.ChangeState( m_Climber.ClimberWinchStop, m_Climber.GrabMaintain );
  }

// ****************************************************************************

  // Update all subsystems
  m_Drivetrain.Update( GetPeriod(), m_fieldRelative );
  m_Climber.Update();
  m_Elevator.Update();
  m_Claw.Update( m_Elevator.ismoving() );
}



#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
