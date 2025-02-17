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
#include <rev/SparkMax.h>
using namespace rev::spark;

class Elevator
{
 public:
    typedef enum ElevatorState_e
    {
        ElevatorStartingConfig,
        ElevatorAlgaeFloor,
        ElevatorAlgaeProcessor,
        ElevatorAlgaeReefLow,
        ElevatorAlgaeReefHigh,
        ElevatorCoralL1,
        ElevatorCoralL2,
        ElevatorCoralL3,
        ElevatorCoralL4
    } ElevatorState_t;

    void Init();
    void Update();
    void ChangeState( ElevatorState_t state );
    void ManualControl( double speed );
    void UpdateSmartDashboardData();

 private:
    SparkMax m_Motor0{ Constants::kElevator0ID, SparkLowLevel::MotorType::kBrushless };
    SparkMax m_Motor1{ Constants::kElevator1ID, SparkLowLevel::MotorType::kBrushless };
    SparkClosedLoopController m_closedLoopController = m_Motor0.GetClosedLoopController();
    SparkRelativeEncoder m_Motor0Encoder = m_Motor0.GetEncoder();


};
/*doalways: AmogusDance
fofever: AmogusDanceBigFunny
never: AmogusDanceStopBeFunny*/