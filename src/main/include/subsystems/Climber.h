// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <Phoenix5.h>
#include <Constants.h>
#include <frc/DigitalInput.h>
#include <frc/Encoder.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/PIDController.h>
#include <units/angular_velocity.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include "units/angular_acceleration.h"
#include <ctre/phoenix/motorcontrol/can/WPI_TalonSRX.h>
#include <frc/DutyCycleEncoder.h>
#include <rev/SparkMax.h>
#include <rev/config/SparkMaxConfig.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>
#include <frc/Timer.h>

class Climber : public frc2::SubsystemBase {
 public:

  typedef enum ClimberWinchState_e
  {
      ClimberWinchOut,
      ClimberWinchIn,
      ClimberWinchStop,
      ClimberWinchCalibrate,
      ClimberWinchInManual,
      ClimberWinchOutManual,
      ClimberWinchMaintain
  } ClimberWinchState_t;

      typedef enum ClimberGrabberState_e
  {
      GrabVertical,
      GrabHorizontal,
      GrabStop,
      GrabMaintain
  } ClimberGrabberState_t;
  
  frc2::CommandPtr ChangeStateCommand( ClimberWinchState_t Cstate, ClimberGrabberState_t Gstate );

  Climber();
  void TeleopInit();
  void ManualControl();
  void ChangeState( ClimberWinchState_t Cstate, ClimberGrabberState_t Gstate );
  void UpdateSmartDashboardData( );


  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
    frc::DigitalInput winch_limit {Constants::kClimbSwitchDIO};
    ctre::phoenix::motorcontrol::can::WPI_TalonSRX m_ClimbMotorEast {Constants::kClimberEastID};
    ctre::phoenix::motorcontrol::can::WPI_TalonSRX m_ClimbMotorWest {Constants::kClimberWestID};
    frc::Encoder m_climberEncoderWest {Constants::kWestEncoderLDIO,Constants::kWestEncoderHDIO};
    frc::Encoder m_climberEncoderEast {Constants::kEastEncoderLDIO,Constants::kEastEncoderHDIO};
    rev::spark::SparkMax m_CageGrabberMotor { Constants::kCageGrabberID, rev::spark::SparkLowLevel::MotorType::kBrushless };

    bool winch_calibrated = false;
    long maxClimbWest = Constants::kEncClimbUp;
    long maxClimbEast = -(Constants::kEncClimbUp);
    ClimberWinchState_t m_ClimberWinchState = ClimberWinchStop;
    ClimberGrabberState_t m_ClimberGrabberState = GrabStop;

    rev::spark::SparkRelativeEncoder m_CageEncoder = m_CageGrabberMotor.GetEncoder();
    frc::PIDController m_GrabberPid {Constants::kGrabberPidP, Constants::kGrabberPidI, Constants::kGrabberPidD};

    frc::Timer grabtimer;
};
