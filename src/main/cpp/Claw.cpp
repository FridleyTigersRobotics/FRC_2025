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
        .SetIdleMode(SparkMaxConfig::IdleMode::kBrake)
        .VoltageCompensation(12.0)
        .SmartCurrentLimit(20, 20);

    // config_CoralAngleMotor.encoder
        // .PositionConversionFactor( 1.0/40.0 )
        // .VelocityConversionFactor( 1.0/40.0 );

    config_CoralAngleMotor.closedLoop
        .SetFeedbackSensor(ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed
        // loop slot, as it will default to slot 0.
        .P(1.0)
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
        .MaxVelocity(1000)
        .MaxAcceleration(1000)
        .AllowedClosedLoopError(1)
        // Set MAXMotion parameters for velocity control in slot 1
        .MaxAcceleration(500, ClosedLoopSlot::kSlot1)
        .MaxVelocity(6000, ClosedLoopSlot::kSlot1)
        .AllowedClosedLoopError(1, ClosedLoopSlot::kSlot1);




    config_AlgaeAngleMotor
        .SetIdleMode(SparkMaxConfig::IdleMode::kBrake)
        .VoltageCompensation(12.0)
        .SmartCurrentLimit(4, 4);


#if 0
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
#endif

    m_CoralAngleMotor.Configure( config_CoralAngleMotor, SparkMax::ResetMode::kResetSafeParameters, SparkMax::PersistMode::kPersistParameters );
    m_AlgaeAngleMotor.Configure( config_AlgaeAngleMotor, SparkMax::ResetMode::kResetSafeParameters, SparkMax::PersistMode::kPersistParameters );
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
                //if ( m_CoralEncoder.Get() < m_CoralEncoderBottom )
                {
                    //coralMotorSpeed = -0.5;
                    m_CoralTargetPosition = 0.0;
                    m_CoralMotorClosedLoopController.SetReference(
                        0.0, SparkMax::ControlType::kMAXMotionPositionControl,
                        ClosedLoopSlot::kSlot0);

                }
                /*else{
                m_CoralAngleMotor.Set( 0.0 );
                }*/
                break;
            }
            case CoralClawUp:
            {
                //if ( m_CoralEncoder.Get() > m_CoralEncoderTop )
                {
                    //coralMotorSpeed = 0.5;
                    m_CoralTargetPosition = 40.0 * (m_CoralEncoderBottom - m_CoralEncoderTop);
                    m_CoralMotorClosedLoopController.SetReference(
                        40.0 * (m_CoralEncoderBottom - m_CoralEncoderTop), SparkMax::ControlType::kMAXMotionPositionControl,
                        ClosedLoopSlot::kSlot0);
                }
                /*else{
                m_CoralAngleMotor.Set( 0.0 );
                }*/
                break;
            }
            case CoralClawStop:
            default:
            {
                coralMotorSpeed = 0.0;
                m_CoralAngleMotor.Set( coralMotorSpeed );
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
                if ( m_AlgaeEncoder.Get() > m_AlgaeEncoderBottom )
                {
                    algaeMotorSpeed = 0.5;
                }
                break;
            }
            case AlgaeClawUp:
            {
                if ( m_AlgaeEncoder.Get() < m_AlgaeEncoderTop )
                {
                    algaeMotorSpeed = -0.5;
                }
                break;
            }
            case AlgaeClawStop:
            default:
            {
                algaeMotorSpeed = 0.0;
                break;
            }
        }

        m_AlgaeAngleMotor.Set( algaeMotorSpeed );
    }

}


// ****************************************************************************
void Claw::ChangeState( CoralClawState_t coralState )
{
    m_coralState = coralState;
}


// ****************************************************************************
void Claw::UpdateSmartDashboardData( )
{
    frc::SmartDashboard::PutNumber("Intake: Algae Encoder Value", m_AlgaeEncoder.Get());
    frc::SmartDashboard::PutNumber("Intake: Coral Encoder Value", m_CoralEncoder.Get());

    frc::SmartDashboard::PutNumber("Intake: Coral Motor Encoder Value", m_CoralMotorEncoder.GetPosition());
    frc::SmartDashboard::PutNumber("Intake: Coral Target Value", m_CoralTargetPosition);
}
