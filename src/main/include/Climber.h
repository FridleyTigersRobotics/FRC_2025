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


class Climber
{
 public:
    typedef enum ClimberState_e
    {
        ClimberDown,
        ClimberUp,
        ClimberStop,
        ClimberReset
    } ClimberState_t;

    

    void Init();
    void TeleopInit();
    void Update();
    void ChangeState( ClimberState_t state );
    void ManualControl( double climbSpeed);
    void UpdateSmartDashboardData();

 private:
    frc::DigitalInput winch_limit {Constants::kClimbSwitchDIO};
    ctre::phoenix::motorcontrol::can::WPI_TalonSRX m_ClimbMotorEast {Constants::kClimberEastID};
    ctre::phoenix::motorcontrol::can::WPI_TalonSRX m_ClimbMotorWest {Constants::kClimberWestID};
    frc::Encoder m_climberEncoderWest {Constants::kWestEncoderLDIO,Constants::kWestEncoderHDIO};
    frc::Encoder m_climberEncoderEast {Constants::kEastEncoderLDIO,Constants::kEastEncoderHDIO};

    double m_ClimbSpeed = 0;
    bool m_ManualClimbControl = true;
    bool winch_calibrated = false;
    long maxClimbWest = Constants::kEncClimbUp;
    long maxClimbEast = -(Constants::kEncClimbUp);
    ClimberState_t m_ClimberState = ClimberStop;
};
