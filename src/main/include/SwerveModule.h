#pragma once

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#include <rev/SparkMax.h>
using namespace rev::spark;
#include <numbers>
#include <frc/AnalogEncoder.h>
#include <frc/Encoder.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <units/angular_velocity.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <rev/sparkmax.h>



class SwerveModule {
 public:
  SwerveModule(
    int driveMotorChannel, 
    int turningMotorChannel,
    int turningEncoderChannel,
    const double turningEncoderOffset,
    units::meters_per_second_t maxSpeed,
    std::string name
  );
  frc::SwerveModuleState GetState() const;
  frc::SwerveModulePosition GetPosition() const;
  void SetDesiredState( frc::SwerveModuleState& state );

 private:
  SparkMax m_driveMotor;
  SparkMax m_turningMotor;
  frc::AnalogEncoder m_turningAbsoluteEncoder;

  std::string m_driveSpeedName;
  std::string m_driveAngleName;
  std::string m_driveRawAngleName;
  units::meters_per_second_t m_maxSpeed{ 0.0 };
  SparkRelativeEncoder m_driveMotorEncoder;
  SparkRelativeEncoder m_turnMotorEncoder;

  static constexpr auto kModuleMaxAngularVelocity =
      std::numbers::pi * 100_rad_per_s;  // radians per second

  static constexpr auto kModuleMaxAngularAcceleration =
      std::numbers::pi * 200_rad_per_s / 1_s;  // radians per second^2

  static constexpr double m_metersPerInch     = 0.0254;
  static constexpr double m_driveGearRatio    = 8.14; // Drivegear ratio of our SDS MK4 L1 module.
  static constexpr double m_wheelDiameterInch = 4.0;  // inches
  static constexpr double m_drivePositonConversionFactor = m_wheelDiameterInch * m_metersPerInch * std::numbers::pi / m_driveGearRatio;
  // Velocity is returned in rotations per minute, convert to meters per second.
  static constexpr double m_driveVelocityConversionFactor = m_drivePositonConversionFactor / 60.0;

  static constexpr double m_turnGearRatio    = 8.14; // Turning gear ratio of our SDS MK4 L1 module.
  static constexpr double m_turnPositonConversionFactor = 2 * std::numbers::pi / m_turnGearRatio; // 2pi per rev
  // Velocity is returned in rotations per minute, convert to 2pi per second.
  static constexpr double m_turnVelocityConversionFactor = m_turnPositonConversionFactor / 60.0;


  frc::ProfiledPIDController<units::radians> m_turningPIDController{
      1.0,
      0.0,
      0.0,
      {kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration}};

};