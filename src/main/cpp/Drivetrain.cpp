// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Drivetrain.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <fmt/printf.h>
#include <math.h>


void Drivetrain::Update( units::second_t period, bool fieldRelative ) 
{
  frc::ChassisSpeeds ChassisSpeedsToUse;
  frc::SmartDashboard::PutNumber("spigyro angle", spigyro.GetAngle());
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
  #if 1
  frc::SmartDashboard::PutNumber( "Drive_X", double{m_odometry.GetEstimatedPosition().X()});
  frc::SmartDashboard::PutNumber( "Drive_y", double{m_odometry.GetEstimatedPosition().Y()});
  frc::SmartDashboard::PutNumber( "Drive_Rot", double{m_odometry.GetEstimatedPosition().Rotation().Degrees()});
  frc::SmartDashboard::PutNumber( "Yawby bobby", m_imu.GetYaw());
  #endif
}


void Drivetrain::UpdateOdometry() {
  m_odometry.Update( frc::Rotation2d{ units::degree_t{ GetYaw() } },
                    { m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
                      m_backLeft.GetPosition(), m_backRight.GetPosition()});
}

void Drivetrain::Init(){
  spigyro.Calibrate();
  spigyro.Reset();
}

void Drivetrain::ResetYaw()
{

}



double Drivetrain::GetYaw(){
  return m_imu.GetYaw();//.9695;
  /*
  double yaw = 0;
  double allthedegs = spigyro.GetAngle();
  // Ensure the angle is within the range [0, 360]
  while (allthedegs < 0) {
        allthedegs += 360;
  }
  while (allthedegs >= 360) {
      allthedegs -= 360;
  }

  // Convert the angle from degrees to radians
  double radian_angle = allthedegs * M_PI / 180.0;

  // Ensure the angle is within the range [-π, π]
  if (radian_angle > M_PI) {
      radian_angle -= 2 * M_PI;
  } else if (radian_angle < -M_PI) {
      radian_angle += 2 * M_PI;
  }

  // Convert the angle from radians back to degrees
  yaw = radian_angle * 180.0 / M_PI;

  return yaw;
  */
}


double Drivetrain::GetAngle(){
  return m_imu.GetAngle();
  //return spigyro.GetAngle();
}
