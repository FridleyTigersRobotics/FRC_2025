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


class Elevator
{
 public:
    typedef enum ElevatorState_e
    {
        ElevatorDown,
        ElevatorUp,
        ElevatorStop
    } ElevatorState_t;

    void Init();
    void Update();
    void ChangeState( ElevatorState_t state );
    void ManualControl();
    void UpdateSmartDashboardData();

 private:
   

    static constexpr double kMaxElevatorHeight = 2.9e5;

  


};
/*doalways: AmogusDance
fofever: AmogusDanceBigFunny
never: AmogusDanceStopBeFunny*/