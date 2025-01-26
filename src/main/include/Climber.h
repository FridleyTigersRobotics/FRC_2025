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


class Climber
{
 public:
    typedef enum ClimberState_e
    {
        ClimberDown,
        ClimberUp,
        ClimberStop
    } ClimberState_t;

    void Init();
    void Update();
    void ChangeState( ClimberState_t state );
    void ManualControl();
    void UpdateSmartDashboardData();

 private:

};
