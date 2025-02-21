#include <Debug.h>
#include <Claw.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/SparkMax.h>
#include <rev/config/SparkMaxConfig.h>
using namespace rev::spark;

void Claw::TeleopInit()
{
    m_CoralMotorEncoder.SetPosition(40.0 * (m_CoralEncoderBottom - m_CoralEncoder.Get()));
}



// ****************************************************************************
void Claw::Init()
{
    SparkMaxConfig config_AlgaeAngleMotor {};
    SparkMaxConfig config_CoralAngleMotor {};
    //SparkMaxConfig config_AlgaeIntakeMotor{};
    //SparkMaxConfig config_CoralIntakeMotor{};


    config_CoralAngleMotor
        .SetIdleMode(SparkMaxConfig::IdleMode::kCoast)
        .VoltageCompensation(12.0)
        .SmartCurrentLimit(10, 10);

    config_CoralAngleMotor.encoder
        .Inverted( true )
        .PositionConversionFactor( 1.0 / 40.0 )
        .VelocityConversionFactor( 1.0 / 40.0* (1.0/60.0) );

    config_CoralAngleMotor.closedLoop
        .SetFeedbackSensor(ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed
        // loop slot, as it will default to slot 0.
        .P(4.0)
        .I(0)
        .D(0)
        .OutputRange(-0.5, 0.5)
        // Set PID values for velocity control in slot 1
        .P(0.0001, ClosedLoopSlot::kSlot1)
        .I(0, ClosedLoopSlot::kSlot1)
        .D(0, ClosedLoopSlot::kSlot1)
        .VelocityFF(1.0 / 5767, ClosedLoopSlot::kSlot1)
        .OutputRange(-0.2, 0.2, ClosedLoopSlot::kSlot1);

    config_CoralAngleMotor.closedLoop
        .maxMotion
        // Set MAXMotion parameters for position control. We don't need to pass
        // a closed loop slot, as it will default to slot 0.
        .MaxVelocity(2000)
        .MaxAcceleration(2000)
        .AllowedClosedLoopError(0.5)
        // Set MAXMotion parameters for velocity control in slot 1
        .MaxAcceleration(500, ClosedLoopSlot::kSlot1)
        .MaxVelocity(6000, ClosedLoopSlot::kSlot1)
        .AllowedClosedLoopError(1, ClosedLoopSlot::kSlot1);




    config_AlgaeAngleMotor
        .SetIdleMode(SparkMaxConfig::IdleMode::kCoast)
        .VoltageCompensation(12.0)
        .SmartCurrentLimit(10, 10);


    config_AlgaeAngleMotor.closedLoop
        .SetFeedbackSensor(ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed
        // loop slot, as it will default to slot 0.
        .P(4.0)
        .I(0)
        .D(0)
        .OutputRange(-0.5, 0.5)
        // Set PID values for velocity control in slot 1
        .P(0.0001, ClosedLoopSlot::kSlot1)
        .I(0, ClosedLoopSlot::kSlot1)
        .D(0, ClosedLoopSlot::kSlot1)
        .VelocityFF(1.0 / 5767, ClosedLoopSlot::kSlot1)
        .OutputRange(-0.2, 0.2, ClosedLoopSlot::kSlot1);

    config_AlgaeAngleMotor.closedLoop
        .maxMotion
        // Set MAXMotion parameters for position control. We don't need to pass
        // a closed loop slot, as it will default to slot 0.
        .MaxVelocity(2000)
        .MaxAcceleration(2000)
        .AllowedClosedLoopError(0.5)
        // Set MAXMotion parameters for velocity control in slot 1
        .MaxAcceleration(500, ClosedLoopSlot::kSlot1)
        .MaxVelocity(6000, ClosedLoopSlot::kSlot1)
        .AllowedClosedLoopError(1, ClosedLoopSlot::kSlot1);

    config_AlgaeAngleMotor.encoder
        .Inverted( true )
        .PositionConversionFactor( 1.0 / 45.0 )
        .VelocityConversionFactor( 1.0 / 45.0 * (1.0/60.0) );


    m_CoralAngleMotor.Configure( config_CoralAngleMotor, SparkMax::ResetMode::kResetSafeParameters, SparkMax::PersistMode::kPersistParameters );
    m_AlgaeAngleMotor.Configure( config_AlgaeAngleMotor, SparkMax::ResetMode::kResetSafeParameters, SparkMax::PersistMode::kPersistParameters );

    m_CoralMotorEncoder.SetPosition( m_CoralEncoder.Get() );
    m_AlgaeMotorEncoder.SetPosition( m_AlgaeEncoder.Get() );


    m_CoralPid.SetIntegratorRange( -0.1, 0.1 ); //stops integrator wind-up
    m_CoralPid.SetTolerance(1.0);
    m_CoralPid.Reset();

    m_AlgaePid.SetIntegratorRange( -0.1, 0.1 ); //stops integrator wind-up
    m_AlgaePid.SetTolerance(1.0);
    m_AlgaePid.Reset();
}


// ****************************************************************************
void Claw::ManualControl( double coralSpeed, double algaeSpeed )
{
    m_CoralMotorSpeed = coralSpeed;
    m_AlgaeMotorSpeed = algaeSpeed;
}


// ****************************************************************************
void Claw::Update()
{
    if ( m_ManualCoralControl )
    {
        m_CoralAngleMotor.Set( m_CoralMotorSpeed );
    }
    else
    {
        double coralMotorSpeed = 0.0;

        switch ( m_coralState )
        {
            case CoralClawDown:
            {
                m_CoralTargetPosition = 0.0;
                break;
            }
            case CoralClawUp:
            {
                m_CoralTargetPosition = 0.0;
                break;
            }
            case CoralClawStop:
            default:
            {
                m_CoralTargetPosition = 0.0;
                break;
            }
        }
    }

    if ( m_ManualAlgaeControl )
    {
        m_AlgaeAngleMotor.Set( m_AlgaeMotorSpeed );
    }
    else
    {
        double algaeMotorSpeed = 0.0;

        switch ( m_algaeState )
        {
            case AlgaeClawDown:
            {
                m_AlgaeTargetPosition = 0.0;
                break;
            }
            case AlgaeClawUp:
            {
                m_AlgaeTargetPosition = 0.0;
                break;
            }
            case AlgaeClawStop:
            default:
            {
                m_AlgaeTargetPosition = 0.0;
                break;
            }
        }

        m_AlgaeAngleMotor.Set( algaeMotorSpeed );
    }

    //if ( m_CoralEncoder.Get() > m_CoralEncoderTop )
    {
        m_CoralMotorClosedLoopController.SetReference(
            m_CoralTargetPosition, 
            SparkMax::ControlType::kMAXMotionPositionControl,
            ClosedLoopSlot::kSlot0
        );
    }

    //m_CoralPid.SetSetpoint(???);
    //double coralMotorValue = m_CoralPid.Calculate(m_CoralEncoder.GetPosition());
    //coralMotorValue = std::clamp(coralMotorValue, -0.5, 0.5 );
    //m_CoralAngleMotor.Set(coralMotorValue);


    //if ( m_CoralEncoder.Get() > m_CoralEncoderTop )
    {
        m_AlgaeMotorClosedLoopController.SetReference(
            m_AlgaeTargetPosition, 
            SparkMax::ControlType::kMAXMotionPositionControl,
            ClosedLoopSlot::kSlot0
        );
    }

    //m_AlgaePid.SetSetpoint(???);
    //double algaeMotorValue = m_AlgaePid.Calculate(m_AlgaeEncoder.GetPosition());
    //algaeMotorValue = std::clamp(algaeMotorValue, -0.5, 0.5 );
    //m_AlgaeAngleMotor.Set(algaeMotorValue);
}


// ****************************************************************************
void Claw::ChangeState( CoralClawState_t coralState, AlgaeClawState_t algaeState )
{
    m_coralState = coralState;
    m_algaeState = algaeState;
}


// ****************************************************************************
void Claw::UpdateSmartDashboardData( )
{
    frc::SmartDashboard::PutNumber("Intake: Algae Encoder Value", m_AlgaeEncoder.Get());
    frc::SmartDashboard::PutNumber("Intake: Coral Encoder Value", m_CoralEncoder.Get());

    frc::SmartDashboard::PutNumber("Intake: Algae Motor Encoder Value", m_AlgaeMotorEncoder.GetPosition());
    frc::SmartDashboard::PutNumber("Intake: Coral Motor Encoder Value", m_CoralMotorEncoder.GetPosition());

    frc::SmartDashboard::PutNumber("Intake: Coral Motor Encoder Value", m_CoralMotorEncoder.GetPosition());
    frc::SmartDashboard::PutNumber("Intake: Coral Target Value", m_CoralTargetPosition);
}
