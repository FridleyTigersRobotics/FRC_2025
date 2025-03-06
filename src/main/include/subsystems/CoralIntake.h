// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <Phoenix5.h>
#include <Constants.h>
#include <frc/DigitalInput.h>
#include <frc/AnalogInput.h>
#include <frc/Encoder.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/PIDController.h>
#include <units/angular_velocity.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include "units/angular_acceleration.h"
#include <frc/DutyCycleEncoder.h>
#include <rev/SparkMax.h>
#include <rev/config/SparkMaxConfig.h>
using namespace rev::spark;
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>
#include "subsystems/Elevator.h"
#include <NavX_Utilities.h>

class CoralIntake : public frc2::SubsystemBase {
 public:
    typedef enum CoralAngleState_e
    {
        AngleUp,
        AngleDn,
        AnglePlaceCoral,
        AnglePlaceTopCoral,
        AngleStop,
        AngleHorizontal,
        AngleMaintain
    } CoralAngleState_t;

    typedef enum coralIntakeState_e
    {
        intakeStop,
        intakeIntake,
        intakeReverse,
        intakeMaintain
    } CoralIntakeState_t;

    frc2::CommandPtr ChangeStateCommand( CoralAngleState_t Astate, CoralIntakeState_t Istate );

    void Init();
    void TeleopInit();
    void Update( bool elevatormoving );
    void ChangeState( CoralAngleState_t Astate, CoralIntakeState_t Istate );
  
    void ManualControl();
    void UpdateSmartDashboardData();

    CoralIntake( Elevator const &Elevator );

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
    Elevator const & m_Elevator;
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
    SparkMax m_CoralAngleMotor { Constants::kCoralAngleID, SparkLowLevel::MotorType::kBrushless };
    SparkMax m_CoralIntakeMotor { Constants::kCoralIntakeID, SparkLowLevel::MotorType::kBrushless };
    SparkRelativeEncoder m_CoralIntakeEncoder = m_CoralIntakeMotor.GetEncoder();

    CoralAngleState_t m_coralAngleState = AngleStop;
    CoralIntakeState_t m_coralIntakeState = intakeStop;

    bool m_ManualCoralControl = false;
    
    frc::DutyCycleEncoder m_CoralAngleEncoder { Constants::kCoralEncoderDIO };
    frc::PIDController m_CoralAnglePid {Constants::kCoralAnglePidP, Constants::kCoralAnglePidI, Constants::kCoralAnglePidD};
    frc::AnalogInput m_CoralDetector{GetChannelFromPin(AnalogIn, 0)};
};
