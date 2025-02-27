// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Drivetrain.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <fmt/printf.h>
#include <math.h>
#include <frc/TimedRobot.h>



Drivetrain::Drivetrain() 
{
    m_imu.ResetDisplacement(); 
    m_imu.Reset();
}

void Drivetrain::SetFieldRelative( bool fieldRelative )
{
  m_fieldRelative = fieldRelative;
}



void Drivetrain::Periodic() 
{
  units::second_t period{ frc::TimedRobot::kDefaultPeriod };
  frc::ChassisSpeeds ChassisSpeedsToUse;
  m_frontLeft.UpdateEncoders();
  m_frontRight.UpdateEncoders();
  m_backLeft.UpdateEncoders();
  m_backRight.UpdateEncoders();

  if ( m_fieldRelative )
  {
    ChassisSpeedsToUse = \
      frc::ChassisSpeeds::FromFieldRelativeSpeeds(
        m_xSpeed, 
        m_ySpeed, 
        m_rot,
        frc::Rotation2d{ units::degree_t { -GetYaw() }}
      );
  }
  else
  {
    ChassisSpeedsToUse = frc::ChassisSpeeds{ m_xSpeed, m_ySpeed, m_rot };
  }
        frc::SmartDashboard::PutNumber("THE ROT", float{m_rot});
  frc::ChassisSpeeds ChassisSpeeds = frc::ChassisSpeeds::Discretize( ChassisSpeedsToUse, period );

  auto states = m_kinematics.ToSwerveModuleStates( ChassisSpeeds );

  m_kinematics.DesaturateWheelSpeeds( &states, kMaxSpeed );

  auto [fl, fr, bl, br] = states;

  
  //double minimumSpeed = 0.01;

  // Check if the wheels don't have a drive velocity to maintain the current wheel orientation.
  bool hasVelocity = true;/*
    fabs( double{fl.speed} ) <= minimumSpeed || 
    fabs( double{fr.speed} ) <= minimumSpeed || 
    fabs( double{bl.speed} ) <= minimumSpeed || 
    fabs( double{br.speed} ) <= minimumSpeed;*/

  frc::SmartDashboard::PutNumber( "fl.speed", double{fl.speed} );
  frc::SmartDashboard::PutNumber( "fr.speed", double{fr.speed} );
  frc::SmartDashboard::PutNumber( "bl.speed", double{bl.speed} );
  frc::SmartDashboard::PutNumber( "br.speed", double{br.speed} );

  if ( !hasVelocity )
  {
    fl.angle = m_frontLeft.GetState().angle;
    fr.angle = m_frontRight.GetState().angle;
    bl.angle = m_backLeft.GetState().angle;
    br.angle = m_backRight.GetState().angle;
  }

  m_frontLeft.SetDesiredState(fl);
  m_frontRight.SetDesiredState(fr);
  m_backLeft.SetDesiredState(bl);
  m_backRight.SetDesiredState(br);
  UpdateSmartDashboardData();
}

void Drivetrain::drive(units::meters_per_second_t xSpeed,
                       units::meters_per_second_t ySpeed,
                       units::radians_per_second_t rot, bool fieldRelative,
                       units::second_t period) 
                       {
  auto states =
      m_kinematics.ToSwerveModuleStates(frc::ChassisSpeeds::Discretize(
          fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                              xSpeed, ySpeed, rot,
                              m_odometry.GetEstimatedPosition().Rotation())
                        : frc::ChassisSpeeds{xSpeed, ySpeed, rot},
          period));
          
          }



void Drivetrain::SetSpeeds(
  units::meters_per_second_t  xSpeed,
  units::meters_per_second_t  ySpeed, 
  units::radians_per_second_t rot
)
{
  m_xSpeed = xSpeed;
  m_ySpeed = ySpeed;
  m_rot    = rot;
}


void Drivetrain::UpdateSmartDashboardData()
{
  #if 1
  frc::SmartDashboard::PutNumber( "Drive_X", double{m_odometry.GetEstimatedPosition().X()});
  frc::SmartDashboard::PutNumber( "Drive_y", double{m_odometry.GetEstimatedPosition().Y()});
  frc::SmartDashboard::PutNumber( "Drive_Rot", double{m_odometry.GetEstimatedPosition().Rotation().Degrees()});
  frc::SmartDashboard::PutNumber( "Yawby bobby", m_imu.GetYaw());
  #endif
}

void Drivetrain::TeleopInit()
{

}


void Drivetrain::UpdateOdometry() {
  m_odometry.Update( frc::Rotation2d{ units::degree_t{ GetYaw() } },
                    { m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
                      m_backLeft.GetPosition(), m_backRight.GetPosition()});
}

void Drivetrain::Init(){
}

void Drivetrain::ResetYaw()
{

}



double Drivetrain::GetYaw(){
  return m_imu.GetYaw()*.9695;
}


double Drivetrain::GetAngle(){
  return m_imu.GetAngle();
}



