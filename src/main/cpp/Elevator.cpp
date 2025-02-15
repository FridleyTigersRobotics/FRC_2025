#include <Debug.h>
#include <Elevator.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <rev/SparkMax.h>
#include <rev/config/SparkMaxConfig.h>
using namespace rev::spark;

// ****************************************************************************
void Elevator::Init()
{
    SparkMaxConfig configMotor0{};
    SparkMaxConfig configMotor1{};

    configMotor0
        .SetIdleMode(SparkMaxConfig::IdleMode::kBrake)
        .VoltageCompensation(12.0)
        .SmartCurrentLimit(20);

    configMotor0.closedLoop
        .SetFeedbackSensor(ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed
        // loop slot, as it will default to slot 0.
        .P(0.1)
        .I(0)
        .D(0)
        .OutputRange(-1, 1)
        // Set PID values for velocity control in slot 1
        .P(0.0001, ClosedLoopSlot::kSlot1)
        .I(0, ClosedLoopSlot::kSlot1)
        .D(0, ClosedLoopSlot::kSlot1)
        .VelocityFF(1.0 / 5767, ClosedLoopSlot::kSlot1)
        .OutputRange(-1, 1, ClosedLoopSlot::kSlot1);

    configMotor0.closedLoop
        .maxMotion
        // Set MAXMotion parameters for position control. We don't need to pass
        // a closed loop slot, as it will default to slot 0.
        .MaxVelocity(1000)
        .MaxAcceleration(1000)
        .AllowedClosedLoopError(1)
        // Set MAXMotion parameters for velocity control in slot 1
        .MaxAcceleration(500, ClosedLoopSlot::kSlot1)
        .MaxVelocity(6000, ClosedLoopSlot::kSlot1)
        .AllowedClosedLoopError(1, ClosedLoopSlot::kSlot1);


    configMotor1
        .Follow( Constants::kElevatorMotor0CanID, true ) // Inverted
        .SetIdleMode(SparkMaxConfig::IdleMode::kBrake)
        .VoltageCompensation(12.0)
        .SmartCurrentLimit(20);


    m_Motor0.Configure( configMotor0, SparkMax::ResetMode::kResetSafeParameters, SparkMax::PersistMode::kPersistParameters );
    m_Motor1.Configure( configMotor1, SparkMax::ResetMode::kResetSafeParameters, SparkMax::PersistMode::kPersistParameters );
}


// ****************************************************************************
void Elevator::Update()
{
  if (frc::SmartDashboard::GetBoolean("Control Mode", false)) {
    /*
     * Get the target velocity from SmartDashboard and set it as the setpoint
     * for the closed loop controller with MAXMotionVelocityControl as the
     * control type.
     */
    double targetVelocity =
        frc::SmartDashboard::GetNumber("Target Velocity", 0);
    m_closedLoopController.SetReference(
        targetVelocity, SparkMax::ControlType::kMAXMotionVelocityControl,
        ClosedLoopSlot::kSlot1);
  } else {
    /*
     * Get the target position from SmartDashboard and set it as the setpoint
     * for the closed loop controller with MAXMotionPositionControl as the
     * control type.
     */
    double targetPosition =
        frc::SmartDashboard::GetNumber("Target Position", 0);
    m_closedLoopController.SetReference(
        targetPosition, SparkMax::ControlType::kMAXMotionPositionControl,
        ClosedLoopSlot::kSlot0);
  }
}


// ****************************************************************************
void Elevator::ChangeState( ElevatorState_t state )
{
   
}


// ****************************************************************************
void Elevator::UpdateSmartDashboardData( )
{
  frc::SmartDashboard::PutNumber("Motor 0 Actual Position", m_Motor0Encoder.GetPosition());
  frc::SmartDashboard::PutNumber("Motor 0 Actual Velocity", m_Motor0Encoder.GetVelocity());

  if (frc::SmartDashboard::GetBoolean("Reset Encoder", false)) {
    frc::SmartDashboard::PutBoolean("Reset Encoder", false);
    // Reset the encoder position to 0
    m_Motor0Encoder.SetPosition(0);
  }

}


// ****************************************************************************
void Elevator::ManualControl( double speed )
{

}

