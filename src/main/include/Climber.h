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


#define Climber_MANUAL_CONTROL       ( 0 )
#define Climber_ENCODER_SYNC_ENABLED ( 0 )
#define Climber_ENCODER_SYNC_TRACK_L ( 0 )
#define Climber_GYRO_ENABLED         ( 0 )


class Climber
{
 public:
    typedef enum ClimberState_e
    {
        ClimberDown,
        ClimberUp,
        ClimberStop
    } ClimberState_t;

    void initClimber();
    void updateClimber (/*Me when the me when... *Literally combusts* */);
    void ChangeClimberState( ClimberState_t ClimberState );
    void manualControl( double speedL, double speedR );
    void UpdateSmartDashboardData();
    void UpdateRoll( double roll );

 private:
   


    static constexpr double kMaxClimberHeight = 2.9e5;

  


};
/*doalways: AmogusDance
fofever: AmogusDanceBigFunny
never: AmogusDanceStopBeFunny*/