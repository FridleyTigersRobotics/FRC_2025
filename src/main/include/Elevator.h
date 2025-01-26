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


#define Elevator_MANUAL_CONTROL       ( 0 )
#define Elevator_ENCODER_SYNC_ENABLED ( 0 )
#define Elevator_ENCODER_SYNC_TRACK_L ( 0 )
#define Elevator_GYRO_ENABLED         ( 0 )


class Elevator
{
 public:
    typedef enum ElevatorState_e
    {
        ElevatorDown,
        ElevatorUp,
        ElevatorStop
    } ElevatorState_t;

    void initElevator();
    void updateElevator (/*Me when the me when... *Literally combusts* */);
    void ChangeElevatorState( ElevatorState_t ElevatorState );
    void manualControl( double speedL, double speedR );
    void UpdateSmartDashboardData();
    void UpdateRoll( double roll );

 private:
   

    static constexpr double kMaxElevatorHeight = 2.9e5;

  


};
/*doalways: AmogusDance
fofever: AmogusDanceBigFunny
never: AmogusDanceStopBeFunny*/