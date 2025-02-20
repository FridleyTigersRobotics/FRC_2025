// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SwerveModule.h"

#include <numbers>
#include <frc/geometry/Rotation2d.h>
#include <string>
#include <frc/smartdashboard/SmartDashboard.h>

#include <rev/config/SparkMaxConfig.h>
using namespace rev::spark;

SwerveModule::SwerveModule(
    int    const driveMotorCanId,
    int    const turningMotorCanId,
    int    const turningEncoderAnalogChannel,
    double const turningEncoderOffset,
    units::meters_per_second_t maxSpeed,
    std::string name
)
    : m_driveMotor            ( driveMotorCanId,   SparkLowLevel::MotorType::kBrushless ),
      m_turningMotor          ( turningMotorCanId, SparkLowLevel::MotorType::kBrushless ),
      m_turningAbsoluteEncoder( turningEncoderAnalogChannel ),
      m_driveSpeedName        ( "DriveSpeed_" + name ),
      m_driveAngleName        ( "Angle_"      + name ),
      m_driveRawAngleName     ( "RawAngle_"   + name ),
      m_maxSpeed              ( maxSpeed ),
      m_driveMotorEncoder     ( m_driveMotor.GetEncoder() ) ,
      m_turnMotorEncoder      ( m_turningMotor.GetEncoder() )
{
    // Limit the PID Controller's input range between 0 and 2*pi and set the input
    // to be continuous.
    m_turningPIDController.EnableContinuousInput(
      units::radian_t{ 0 }, 
      units::radian_t{ 2 * std::numbers::pi } );

    SparkMaxConfig configDrive{};
    configDrive
        .Inverted(false)
        .SetIdleMode(SparkMaxConfig::IdleMode::kBrake)
        .VoltageCompensation(12.0)
        .SmartCurrentLimit(30, 60);
    configDrive.encoder
        .PositionConversionFactor( m_drivePositonConversionFactor )
        .VelocityConversionFactor( m_driveVelocityConversionFactor );


    SparkMaxConfig configTurn{};
    configTurn
        .Inverted(false)
        .SetIdleMode(SparkMaxConfig::IdleMode::kBrake)
        .VoltageCompensation(12.0)
        .SmartCurrentLimit(20);
    configTurn.encoder
        .PositionConversionFactor( m_turnPositonConversionFactor )
        .VelocityConversionFactor( m_turnVelocityConversionFactor );

    m_driveMotor  .Configure(configDrive, SparkMax::ResetMode::kResetSafeParameters, SparkMax::PersistMode::kPersistParameters);
    m_turningMotor.Configure(configTurn,  SparkMax::ResetMode::kResetSafeParameters, SparkMax::PersistMode::kPersistParameters);

    m_driveMotorEncoder.SetPosition( 0 );
    m_driveMotorEncoder.SetPosition( turningEncoderOffset );
}

frc::SwerveModuleState SwerveModule::GetState() const {
  return { units::meters_per_second_t{ m_driveMotorEncoder.GetVelocity()   },
           units::radian_t           { m_driveMotorEncoder.GetPosition() } };
}

frc::SwerveModulePosition SwerveModule::GetPosition() const {
  return { units::meter_t  { m_driveMotorEncoder.GetPosition()   },
           units::radian_t { m_driveMotorEncoder.GetPosition() } };
}



void SwerveModule::SetDesiredState(
  frc::SwerveModuleState& referenceState
) 
{
  units::radian_t encoderRads{m_driveMotorEncoder.GetPosition()};
  frc::Rotation2d encoderRotation{encoderRads};

  // Optimize the reference state to avoid spinning further than 90 degrees
  referenceState.Optimize( encoderRotation );
  // Scale speed by cosine of angle error. This scales down movement
  // perpendicular to the desired direction of travel that can occur when
  // modules change directions. This results in smoother driving.
  referenceState.speed *= ( referenceState.angle - encoderRotation ).Cos();

  // Calculate the turning motor output from the turning PID controller.
  const auto turnOutput = m_turningPIDController.Calculate( encoderRads, referenceState.angle.Radians());
  
  double driveMotorOutput = double{ referenceState.speed.value() / m_maxSpeed };

  // Set the motor outputs.
  m_driveMotor.Set( driveMotorOutput );
  m_turningMotor.Set( turnOutput );
}