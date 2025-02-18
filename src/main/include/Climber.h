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


class Climber
{
 public:
    typedef enum ClimberState_e
    {
        ClimberDown,
        ClimberUp,
        ClimberStop,
        ClimberReset,
        GrabSpin
    } ClimberState_t;

    

    void Init();
    void TeleopInit();
    void Update();
    void ChangeState( ClimberState_t state );
    void ManualControl( double climbSpeed, double grabSpeed );
    void UpdateSmartDashboardData();

 private:
    frc::DigitalInput winch_limit {Constants::kClimbSwitchDIO};
    ctre::phoenix::motorcontrol::can::WPI_TalonSRX m_ClimbMotorEast {Constants::kClimberEastID};
    ctre::phoenix::motorcontrol::can::WPI_TalonSRX m_ClimbMotorWest {Constants::kClimberWestID};
    frc::Encoder m_climberEncoderWest {Constants::kWestEncoderLDIO,Constants::kWestEncoderHDIO};
    frc::Encoder m_climberEncoderEast {Constants::kEastEncoderLDIO,Constants::kEastEncoderHDIO};
    rev::spark::SparkMax m_CageGrabberMotor { Constants::kCageGrabberID, rev::spark::SparkLowLevel::MotorType::kBrushless };

    double m_ClimbSpeed = 0;
    double m_GrabSpeed = 0;
    bool m_ManualClimbControl = true;
    bool winch_calibrated = false;
    long maxClimbWest = Constants::kEncClimbUp;
    long maxClimbEast = -(Constants::kEncClimbUp);
    ClimberState_t m_ClimberState = ClimberStop;

    rev::spark::SparkRelativeEncoder m_CageEncoder = m_CageGrabberMotor.GetEncoder();
    frc::PIDController m_GrabberPid {Constants::kGrabberPidP, Constants::kGrabberPidI, Constants::kGrabberPidD};
};
