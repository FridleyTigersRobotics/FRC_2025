// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Drivetrain.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <fmt/printf.h>
#include <math.h>

void Drivetrain::updateDrivetrain( units::second_t period, bool fieldRelative ) 
{
  frc::ChassisSpeeds ChassisSpeedsToUse;
  m_frontLeft.UpdateEncoders();
  m_frontRight.UpdateEncoders();
  m_backLeft.UpdateEncoders();
  m_backRight.UpdateEncoders();

  if ( fieldRelative )
  {
    ChassisSpeedsToUse = \
      frc::ChassisSpeeds::FromFieldRelativeSpeeds(
        m_xSpeed, 
        m_ySpeed, 
        m_rot, 
        frc::Rotation2d{ units::degree_t { 0/*-GetYaw()*/ }}
      );
  }
  else
  {
    ChassisSpeedsToUse = frc::ChassisSpeeds{ m_xSpeed, m_ySpeed, m_rot };
  }

  frc::ChassisSpeeds ChassisSpeeds = frc::ChassisSpeeds::Discretize( ChassisSpeedsToUse, period );
  UpdateOdometry();
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


void Drivetrain::AddToSpeeds(
  units::meters_per_second_t  xSpeed,
  units::meters_per_second_t  ySpeed, 
  units::radians_per_second_t rot
)
{
  m_xSpeed += xSpeed;
  m_ySpeed += ySpeed;
  m_rot    += rot;
}


void Drivetrain::UpdateSmartDashboardData()
{
  #if 0
  frc::SmartDashboard::PutNumber( "Drive_X", double{m_odometry.GetPose().X()} );
  frc::SmartDashboard::PutNumber( "Drive_y", double{m_odometry.GetPose().Y()} );
  frc::SmartDashboard::PutNumber( "Drive_Rot", double{m_odometry.GetPose().Rotation().Degrees()} );
  #endif
}


void Drivetrain::UpdateOdometry() {
  m_odometry.Update( frc::Rotation2d{ units::degree_t{ GetYaw() } },
                    { m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
                      m_backLeft.GetPosition(), m_backRight.GetPosition()});
}

void Drivetrain::Init(){
  //m_YawOffset=0;
  //m_imu.ZeroYaw();
  //m_imu.Reset();
}

void Drivetrain::ResetYaw()
{
  double currentAngle = m_imu.GetAngle() + m_YawOffset;
  double unwrappedAngle = std::remainder( currentAngle, 360.0 );
  m_YawOffset=unwrappedAngle;
  m_imu.ZeroYaw();
  m_imu.Reset();
}



double Drivetrain::GetYaw(){
  return ((m_imu.GetYaw()) + m_YawOffset);
}


double Drivetrain::GetAngle(){
  return ((m_imu.GetAngle()) + m_YawOffset);
}
