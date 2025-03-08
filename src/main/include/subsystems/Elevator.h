// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
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
#include <rev/SparkMax.h>
using namespace rev::spark;



class Elevator : public frc2::SubsystemBase {
 public:
  Elevator();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;



    typedef enum ElevatorState_e
    {
        ElevatorStartingConfig,
        ElevatorCoralIntake,
        ElevatorCoralL1,
        ElevatorCoralL2,
        ElevatorCoralL3,
        ElevatorCoralL4,
        ElevatorStop,
        ElevatorMaintain,
        ElevatorL2PrepBump,
        ElevatorL2EndBump,
        ElevatorL3PrepBump,
        ElevatorL3EndBump
    } ElevatorState_t;

    frc2::CommandPtr ChangeStateCommand( ElevatorState_t ElevatorPosition );

    void Init();
    void TeleopInit();
    void Update();
    void ChangeState( ElevatorState_t ElevatorPosition );
    void ManualControl();
    void UpdateSmartDashboardData();
    bool ismoving() const;
    bool IsNearBottom() const;

 private:
    SparkMax m_Motor0{ Constants::kElevator0ID, SparkLowLevel::MotorType::kBrushless };
    SparkMax m_Motor1{ Constants::kElevator1ID, SparkLowLevel::MotorType::kBrushless };
    SparkRelativeEncoder m_Elevator0Encoder = m_Motor0.GetEncoder();
    SparkRelativeEncoder m_Elevator1Encoder = m_Motor1.GetEncoder();

    ElevatorState_t m_ElevatorState = ElevatorStop;

    frc::PIDController m_ElevatorPid {Constants::kElevatorPidP, Constants::kElevatorPidI, Constants::kElevatorPidD};

};
