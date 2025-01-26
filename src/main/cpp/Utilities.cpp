#include <Debug.h>
#include <Robot.h>
#include <frc/MathUtil.h>
#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <fmt/printf.h>
#include <frc/filter/SlewRateLimiter.h>

#include "Drivetrain.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <string>


// Function from Kauailab Website:
//   https://pdocs.kauailabs.com/navx-mxp/examples/mxp-io-expansion/
int GetAnalogChannelFromPin( int io_pin_number ) {
    //static const int MAX_NAVX_MXP_DIGIO_PIN_NUMBER      = 9;
    static const int MAX_NAVX_MXP_ANALOGIN_PIN_NUMBER   = 3;
    //static const int MAX_NAVX_MXP_ANALOGOUT_PIN_NUMBER  = 1;
    //static const int NUM_ROBORIO_ONBOARD_DIGIO_PINS     = 10;
    //static const int NUM_ROBORIO_ONBOARD_PWM_PINS       = 10;
    static const int NUM_ROBORIO_ONBOARD_ANALOGIN_PINS  = 4;
    int roborio_channel = 0;

    if ( io_pin_number < 0 ) {
        throw std::runtime_error("Error:  navX-MXP I/O Pin #");
    }

    if ( io_pin_number > MAX_NAVX_MXP_ANALOGIN_PIN_NUMBER ) {
        throw new std::runtime_error("Error:  Invalid navX-MXP Analog Input Pin #");
    }
    roborio_channel = io_pin_number + NUM_ROBORIO_ONBOARD_ANALOGIN_PINS;

    return roborio_channel;
}




