// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Drivetrain.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <fmt/printf.h>
#include <math.h>



Drivetrain::Drivetrain( ) 
{
  m_DriveRotatePid.EnableContinuousInput(-180, 180);
  m_imu.ResetDisplacement();
}


void Drivetrain::Drive( double DriveX, double DriveY, double RotX, double RotY, bool fieldRelative  ) 
{
  m_fieldRelative = fieldRelative;
  units::radians_per_second_t rotationSpeed{ 0.0 };
  units::meters_per_second_t  xSpeed = DriveX * Drivetrain::kMaxSpeed;
  units::meters_per_second_t  ySpeed = DriveY * Drivetrain::kMaxSpeed;

  if ( m_fieldRelative )
  {
    double mag   = sqrt( RotX * RotX + RotY * RotY );
    double angle = 180.0 * atan2( RotX , RotY ) / std::numbers::pi;

    frc::SmartDashboard::PutNumber( "InputAngle", double{angle} );
    frc::SmartDashboard::PutNumber( "InputMag", double{mag} );

    if( mag > 0.8 )
    {
      m_DriveTargetAngle = angle;
      m_DriveRotatePid.SetSetpoint( m_DriveTargetAngle );
    }

    double currentAngle           = GetYaw();
    double driveRotSpeedUnClamped = m_DriveRotatePid.Calculate( currentAngle );
    double driveRotSpeed          = std::clamp( driveRotSpeedUnClamped, -1.0, 1.0 );
    // Get the rate of angular rotation. We are inverting this.
    //const auto rotationSpeedUnClamped = ( m_DriveRotatePid.GetError() > 5.0 ) ? ( driveRotSpeed * ( units::radians_per_second_t{0.0} ) ) : ( units::radians_per_second_t{0.0} );
    rotationSpeed =  driveRotSpeed * Drivetrain::kMaxAngularSpeed;

    frc::SmartDashboard::PutNumber( "Err.", m_DriveRotatePid.GetError());
  }
  else
  {
    double driveRotSpeed = RotX;
    rotationSpeed = driveRotSpeed * Drivetrain::kMaxAngularSpeed;
  }

  SetSpeeds( xSpeed, ySpeed, rotationSpeed );
}



void Drivetrain::Update( units::second_t period ) 
{
  frc::ChassisSpeeds ChassisSpeedsToUse;

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
  frc::SmartDashboard::PutNumber( "Drive_xSpeed", double{m_xSpeed} );
  frc::SmartDashboard::PutNumber( "Drive_ySpeed", double{m_ySpeed} );
  frc::SmartDashboard::PutNumber( "Drive_rot",    double{m_rot} );

  frc::SmartDashboard::PutNumber( "Drive_PoseX",   double{m_odometry.GetEstimatedPosition().X()});
  frc::SmartDashboard::PutNumber( "Drive_PoseY",   double{m_odometry.GetEstimatedPosition().Y()});
  frc::SmartDashboard::PutNumber( "Drive_PoseRot", double{m_odometry.GetEstimatedPosition().Rotation().Degrees()});
  frc::SmartDashboard::PutNumber( "Drive_NavxYaw",      m_imu.GetYaw());
  frc::SmartDashboard::PutNumber( "Drive_NavxAccumYaw", m_imu.GetAngle());
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
  return m_imu.GetYaw();
}


double Drivetrain::GetAngle(){
  return m_imu.GetAngle();
}
