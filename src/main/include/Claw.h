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

#define MANUAL_CORAL_CONTROL ( 0 )



class Claw
{
 public:
    typedef enum ClawState_e
    {
        ClawDown,
        ClawUp,
        ClawStop
    } ClawState_t;

    typedef enum CoralClawState_e
    {
        CoralClawDown,
        CoralClawUp,
        CoralClawStop
    } CoralClawState_t;

    void Init();
    void Update();
    void ChangeState( CoralClawState_t coralState );
    void ManualControl( double coralSpeed );
    void UpdateSmartDashboardData();

 private:

    CoralClawState_t m_coralState = CoralClawStop;


    double m_CoralMotorSpeed = 0;



    SparkMax m_AlgaeAngleMotor { Constants::kAlgaeAngleID, SparkLowLevel::MotorType::kBrushless };
    SparkMax m_CoralAngleMotor { Constants::kCoralAgnleID, SparkLowLevel::MotorType::kBrushless };
    SparkMax m_AlgaeIntakeMotor{ Constants::kAlgaeIntakeID, SparkLowLevel::MotorType::kBrushless };
    SparkMax m_CoralIntakeMotor{ Constants::kCoralIntakeID, SparkLowLevel::MotorType::kBrushless };


    constexpr static const double m_AlgaeEncoderMin{ 0.483 };
    constexpr static const double m_AlgaeEncoderMax{ 0.766 };
    constexpr static const double m_CoralEncoderTop{ 0.288 + 0.05 };
    constexpr static const double m_CoralEncoderBottom{ 0.627 - 0.05 };

    frc::DutyCycleEncoder m_AlgaeEncoder { Constants::kAlgaeEncoderDIO };
    frc::DutyCycleEncoder m_CoralEncoder { Constants::kCoralEncoderDIO };
};
