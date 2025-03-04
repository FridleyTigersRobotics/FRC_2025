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
#include <frc/DutyCycleEncoder.h>
#include <rev/SparkMax.h>
#include <rev/config/SparkMaxConfig.h>
using namespace rev::spark;
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>
#include "subsystems/Elevator.h"

class AlgaeIntake : public frc2::SubsystemBase {
 public:
    typedef enum AlgaeAngleState_e
    {
        AngleUp,
        AngleDn,
        AngleStop,
        AngleMaintain
    } AlgaeAngleState_t;

    typedef enum algaeIntakeState_e
    {
        intakeStop,
        intakeIntake,
        intakeReverse,
        intakeMaintain
    } AlgaeIntakeState_t;

    frc2::CommandPtr ChangeStateCommand( AlgaeAngleState_t Astate, AlgaeIntakeState_t Istate );

    void Init();
    void TeleopInit();
    void Update( bool elevatormoving );
    void ChangeState( AlgaeAngleState_t Astate, AlgaeIntakeState_t Istate );
  
    void ManualControl();
    void UpdateSmartDashboardData();

    AlgaeIntake( Elevator const &Elevator );

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
    Elevator const & m_Elevator;
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
    SparkMax m_AlgaeAngleMotor { Constants::kAlgaeAngleID, SparkLowLevel::MotorType::kBrushless };
    SparkMax m_AlgaeIntakeMotor { Constants::kAlgaeIntakeID, SparkLowLevel::MotorType::kBrushless };
    SparkRelativeEncoder m_AlgaeIntakeEncoder = m_AlgaeIntakeMotor.GetEncoder();

    AlgaeAngleState_t m_algaeAngleState = AngleStop;
    AlgaeIntakeState_t m_algaeIntakeState = intakeStop;

    bool m_ManualAlgaeControl = false;
    
    frc::DutyCycleEncoder m_AlgaeAngleEncoder { Constants::kAlgaeEncoderDIO };
    frc::PIDController m_AlgaeAnglePid {Constants::kAlgaeAnglePidP, Constants::kAlgaeAnglePidI, Constants::kAlgaeAnglePidD};
};