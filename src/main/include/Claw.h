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


class Claw
{
 public:
    typedef enum CoralAngleState_e
    {
        AngleUp,
        AngleDn,
        AngleStop
    } CoralAngleState_t;

    typedef enum coralIntakeState_e
    {
        intakeStop,
        intakeIntake,
        intakeReverse
    } CoralIntakeState_t;

    void Init();
    void TeleopInit();
    void Update();
    void ChangeState( CoralAngleState_t Astate, CoralIntakeState_t Istate );
    void ManualControl();
    void UpdateSmartDashboardData();

 private:
    SparkMax m_CoralAngleMotor { Constants::kCoralAngleID, SparkLowLevel::MotorType::kBrushless };
    SparkMax m_CoralIntakeMotor { Constants::kCoralIntakeID, SparkLowLevel::MotorType::kBrushless };
    SparkRelativeEncoder m_CoralIntakeEncoder = m_CoralIntakeMotor.GetEncoder();

    double setvalCoralEncoderTop = Constants::kCoralEncoderTop;
    double setvalCoralEncoderBottom = Constants::kCoralEncoderBottom;
   
    CoralAngleState_t m_coralAngleState = AngleStop;
    CoralIntakeState_t m_coralIntakeState = intakeStop;

    bool m_ManualCoralControl = false;
    
    frc::DutyCycleEncoder m_CoralAngleEncoder { Constants::kCoralEncoderDIO };
    frc::PIDController m_CoralAnglePid {Constants::kCoralAnglePidP, Constants::kCoralAnglePidI, Constants::kCoralAnglePidD};

};
