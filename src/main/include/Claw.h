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
    typedef enum AlgaeClawState_e
    {
        AlgaeUp,
        AlgaeGround,
        AlgaeL1,
        AlgaeL2
    } AlgaeClawState_t;

    typedef enum CoralClawState_e
    {
        CoralUp,
        CoralL1,
        CoralL2
    } CoralClawState_t;

    void Init();
    void TeleopInit();
    void Update();
    void ChangeState( CoralClawState_t coralState, AlgaeClawState_t algaeState );
   void ChangeAIntakeState( double algaeIntake );
   void ChangeCIntakeState( double coralIntake );
    void ManualControl( double coralSpeed, double algaeSpeed );
    void UpdateSmartDashboardData();

 private:
    bool m_ManualCoralControl = false;
    bool m_ManualAlgaeControl = false;

    CoralClawState_t m_coralState = CoralUp;
    AlgaeClawState_t m_algaeState = AlgaeUp;


    double m_CoralMotorSpeed = 0;
    double m_AlgaeMotorSpeed = 0;

    double m_CoralTargetPosition = m_CoralEncoderTop;
    double m_AlgaeTargetPosition = m_AlgaeEncoderTop;

    SparkMax m_AlgaeAngleMotor { Constants::kAlgaeAngleID, SparkLowLevel::MotorType::kBrushless };
    SparkMax m_CoralAngleMotor { Constants::kCoralAgnleID, SparkLowLevel::MotorType::kBrushless };
    SparkMax m_AlgaeIntakeMotor{ Constants::kAlgaeIntakeID, SparkLowLevel::MotorType::kBrushless };
    SparkMax m_CoralIntakeMotor{ Constants::kCoralIntakeID, SparkLowLevel::MotorType::kBrushless };

    SparkClosedLoopController m_CoralMotorClosedLoopController = m_CoralAngleMotor.GetClosedLoopController();
    SparkRelativeEncoder      m_CoralMotorEncoder              = m_CoralAngleMotor.GetEncoder();

    SparkClosedLoopController m_AlgaeMotorClosedLoopController = m_CoralAngleMotor.GetClosedLoopController();
    SparkRelativeEncoder      m_AlgaeMotorEncoder              = m_CoralAngleMotor.GetEncoder();


    constexpr static const double m_AlgaeEncoderBottom{ 0.483 };
    constexpr static const double m_AlgaeEncoderTop{ 0.766 };
    constexpr static const double m_CoralEncoderTop{ 0.288 + 0.05 };
    constexpr static const double m_CoralEncoderBottom{ 0.627 - 0.05 };

    frc::DutyCycleEncoder m_AlgaeEncoder { Constants::kAlgaeEncoderDIO };
    frc::DutyCycleEncoder m_CoralEncoder { Constants::kCoralEncoderDIO };

    frc::PIDController m_CoralPid {2.0, 0.0, 0.0};
    frc::PIDController m_AlgaePid {2.0, 0.0, 0.0};


};
