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


#define Claw_MANUAL_CONTROL       ( 0 )
#define Claw_ENCODER_SYNC_ENABLED ( 0 )
#define Claw_ENCODER_SYNC_TRACK_L ( 0 )
#define Claw_GYRO_ENABLED         ( 0 )


class Claw
{
 public:
    typedef enum ClawState_e
    {
        ClawDown,
        ClawUp,
        ClawStop
    } ClawState_t;

    void initClaw();
    void updateClaw (/*Me when the me when... *Literally combusts* */);
    void ChangeClawState( ClawState_t ClawState );
    void manualControl( double speedL, double speedR );
    void UpdateSmartDashboardData();
    void UpdateRoll( double roll );

 private:
   

    static constexpr double kMaxClawHeight = 2.9e5;

  


};
/*doalways: AmogusDance
fofever: AmogusDanceBigFunny
never: AmogusDanceStopBeFunny*/