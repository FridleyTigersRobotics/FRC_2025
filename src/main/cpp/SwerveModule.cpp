// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SwerveModule.h"

#include <numbers>
//#include <Cameron's Brain> [ERROR:NONEXISTANT] HAHA FUNNY
#include <frc/geometry/Rotation2d.h>
#include <string>
#include <frc/smartdashboard/SmartDashboard.h>

#include <rev/config/SparkMaxConfig.h>
using namespace rev::spark;

#define BURN_FLASH           ( 0 ) 
#define SMART_CURRENT_LIMITS ( 1 )

SwerveModule::SwerveModule(
    const int driveMotorChannel,
    const int turningMotorChannel,
    const int turningEncoderChannel,
    const double turningEncoderOffset,
    units::meters_per_second_t maxSpeed,
    std::string name
)
    : m_driveMotor(driveMotorChannel,SparkLowLevel::MotorType::kBrushless),
      m_turningMotor(turningMotorChannel,SparkLowLevel::MotorType::kBrushless),
      m_turningEncoder(turningEncoderChannel) 
{
    SparkMaxConfig configDrive{};
    SparkMaxConfig configTurn{};
    m_name = name;
    m_driveSpeedName = "DriveSpeed_" + name;
    m_driveAngleName = "Angle_" + name;

    m_driveRawAngleName = "RawAngle_" + name;

  m_maxSpeed = maxSpeed;
  // Set the distance per pulse for the drive encoder. We can simply use the
  // distance traveled for one rotation of the wheel divided by the encoder
  // resolution.
  m_driveEncoder.SetPosition(0); 
  double positonConversionFactor = 3.9*std::numbers::pi/8.14*0.0254;
  m_PositionConversionFactor = positonConversionFactor;
  //m_driveEncoder.SetPositionConversionFactor(positonConversionFactor);
  //m_driveEncoder.SetVelocityConversionFactor((1.0/60.0) * positonConversionFactor);

  m_drivechannel=driveMotorChannel;
  //m_encodername=fmt::sprintf("turnencoder %d",m_drivechannel);
    //m_driveMotor.SetIdleMode(SparkBase::IdleMode::kBrake);
    //m_turningMotor.SetIdleMode(SparkBase::IdleMode::kBrake);
    //m_driveMotor.EnableVoltageCompensation(12.0);
    //m_turningMotor.EnableVoltageCompensation(12.0);

  //m_driveMotor.SetSmartCurrentLimit(30, 60);
  //m_turningMotor.SetSmartCurrentLimit(20);

  // Set the distance (in this case, angle) per pulse for the turning encoder.
  // This is the the angle through an entire rotation (2 * std::numbers::pi)
  // divided by the encoder resolution.
  m_turningEncoder.SetDistancePerRotation( 2 * std::numbers::pi );
  m_turningEncoder.SetPositionOffset( turningEncoderOffset );

  // Limit the PID Controller's input range between -pi and pi and set the input
  // to be continuous.
  //m_turningPIDController.EnableContinuousInput(-units::radian_t{std::numbers::pi}, units::radian_t{std::numbers::pi});
  m_turningPIDController.EnableContinuousInput(units::radian_t{0}, units::radian_t{2.0*std::numbers::pi});



    configDrive
        .Inverted(false)
        .SetIdleMode(SparkMaxConfig::IdleMode::kBrake)
        .VoltageCompensation(12.0)
        .SmartCurrentLimit(20);
    configDrive.encoder
        .PositionConversionFactor( positonConversionFactor )
        .VelocityConversionFactor( (1.0/60.0) * positonConversionFactor );


    configTurn
        .Inverted(false)
        .SetIdleMode(SparkMaxConfig::IdleMode::kCoast)
        .VoltageCompensation(12.0)
        .SmartCurrentLimit(30, 60);
    configTurn.encoder
        .PositionConversionFactor( positonConversionFactor )
        .VelocityConversionFactor( (1.0/60.0) * positonConversionFactor );

    m_driveMotor  .Configure(configDrive, SparkMax::ResetMode::kResetSafeParameters, SparkMax::PersistMode::kPersistParameters);
    m_turningMotor.Configure(configTurn,  SparkMax::ResetMode::kResetSafeParameters, SparkMax::PersistMode::kPersistParameters);


}

frc::SwerveModuleState SwerveModule::GetState() const {
  return {units::meters_per_second_t{m_driveEncoder.GetVelocity()},
          units::radian_t{m_turningEncoder.GetDistance()}};
}

frc::SwerveModulePosition SwerveModule::GetPosition() const {
  return {units::meter_t{m_driveEncoder.GetPosition()},
          units::radian_t{m_turningEncoder.GetDistance()}};
}



void SwerveModule::UpdateEncoders() 
{
  m_turningEncoder.Update();
}


void SwerveModule::SetDesiredState(
    const frc::SwerveModuleState& referenceState
) 
{
  frc::Rotation2d encoderRotation{units::radian_t{m_turningEncoder.GetDistance()}};
  // Optimize the reference state to avoid spinning further than 90 degrees
  auto state = frc::SwerveModuleState::Optimize(referenceState, encoderRotation);

  // Scale speed by cosine of angle error. This scales down movement
  // perpendicular to the desired direction of travel that can occur when
  // modules change directions. This results in smoother driving.
  state.speed *= (state.angle - encoderRotation).Cos();

  // Calculate the turning motor output from the turning PID controller.
  const auto turnOutput = m_turningPIDController.Calculate(
      units::radian_t{m_turningEncoder.GetDistance() }, state.angle.Radians());
  
  double driveMotorOutput = double{state.speed.value() / double{m_maxSpeed}};

  frc::SmartDashboard::PutNumber( m_driveRawAngleName, m_turningEncoder.GetRawPos() );
  frc::SmartDashboard::PutNumber( m_driveAngleName, m_turningEncoder.GetDistance() );
  frc::SmartDashboard::PutNumber( m_driveSpeedName, driveMotorOutput );
   // Set the motor outputs.
   m_driveMotor.Set( driveMotorOutput );
   m_turningMotor.Set( turnOutput );
}