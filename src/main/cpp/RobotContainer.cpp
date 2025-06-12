// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#define UseShuffleboardAPI 0
#define UseElasticNetTable 1

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>

#include "commands/Autos.h"
#include "commands/ExampleCommand.h"
#include <frc2/command/Command.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <frc2/command/RunCommand.h>


#include <frc/TimedRobot.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <memory>
#include <frc2/command/SubsystemBase.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/DriverStation.h>


#include "frc2/command/WaitCommand.h"

#include <fmt/format.h>
#include <wpi/sendable/SendableBuilder.h>

using namespace frc2;


#include <frc/Timer.h>
#include <units/time.h>

#include "frc2/command/Command.h"
#include "frc2/command/CommandHelper.h"


/**
 * A command that does nothing but takes a specified amount of time to finish.
 *
 * This class is provided by the NewCommands VendorDep
 */
class WaitCommand2 : public CommandHelper<Command, WaitCommand2> {
 public:
  /**
   * Creates a new WaitCommand.  This command will do nothing, and end after the
   * specified duration.
   *
   * @param duration the time to wait
   */
  explicit WaitCommand2(units::second_t *duration);

  WaitCommand2(WaitCommand2&& other) = default;

  WaitCommand2(const WaitCommand2& other) = default;

  void Initialize() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  bool RunsWhenDisabled() const override;

  void InitSendable(wpi::SendableBuilder& builder) override;

 protected:
  /// The timer used for waiting.
  frc::Timer m_timer;

 private:
  units::second_t *m_duration;
};


WaitCommand2::WaitCommand2(units::second_t *duration) : m_duration{duration} {
  SetName(fmt::format("{}: {}", GetName(), *duration));
}

void WaitCommand2::Initialize() {
  m_timer.Restart();
}

void WaitCommand2::End(bool interrupted) {
  m_timer.Stop();
}

bool WaitCommand2::IsFinished() {
  return m_timer.HasElapsed(*m_duration);
}

bool WaitCommand2::RunsWhenDisabled() const {
  return true;
}

void WaitCommand2::InitSendable(wpi::SendableBuilder& builder) {
  Command::InitSendable(builder);
  builder.AddDoubleProperty(
      "duration", [this] { return m_duration->value(); }, nullptr);
}




RobotContainer::RobotContainer() : m_Elevator(), m_CoralIntake(m_Elevator), m_AlgaeIntake(m_Elevator) {
  // Initialize all of your commands and subsystems here

  //Autonomous Commands ****************************************************************************
  // Register Named Commands. You must pass either a CommandPtr rvalue or a shared_ptr to the command, not the command directly.
  pathplanner::NamedCommands::registerCommand("ElevatorL1", std::move(frc2::cmd::Parallel(
      m_CoralIntake.ChangeStateCommand( CoralIntake::AngleDn, CoralIntake::intakeStop ),
      m_Elevator.ChangeStateCommand( Elevator::ElevatorCoralL1 ),
      m_AlgaeIntake.ChangeStateCommand( AlgaeIntake::AngleMaintain)
    ))); // <- This example method returns CommandPtr

  pathplanner::NamedCommands::registerCommand("ElevatorL2", std::move(frc2::cmd::Parallel(
      m_CoralIntake.ChangeStateCommand( CoralIntake::AnglePlaceCoral, CoralIntake::intakeStop ),
      m_Elevator.ChangeStateCommand( Elevator::ElevatorCoralL2 ),
      m_AlgaeIntake.ChangeStateCommand( AlgaeIntake::AngleMaintain )
    ))); // <- This example method returns CommandPtr

  pathplanner::NamedCommands::registerCommand("ElevatorL3", std::move(frc2::cmd::Parallel(
      m_CoralIntake.ChangeStateCommand( CoralIntake::AnglePlaceCoral, CoralIntake::intakeStop ),
      m_Elevator.ChangeStateCommand( Elevator::ElevatorCoralL3 ),
      m_AlgaeIntake.ChangeStateCommand( AlgaeIntake::AngleMaintain )
    ))); // <- This example method returns CommandPtr

  pathplanner::NamedCommands::registerCommand("ElevatorL4", std::move(frc2::cmd::Parallel(
      m_CoralIntake.ChangeStateCommand( CoralIntake::AnglePlaceTopCoral, CoralIntake::intakeStop ),
      m_Elevator.ChangeStateCommand( Elevator::ElevatorCoralL4 ),
      m_AlgaeIntake.ChangeStateCommand( AlgaeIntake::AngleMaintain)
    ))); // <- This example method returns CommandPtr

  pathplanner::NamedCommands::registerCommand("ElevatorIntakeCoral", std::move(frc2::cmd::Parallel(
      m_CoralIntake.ChangeStateCommand( CoralIntake::AngleDn, CoralIntake::autoIntakeFwd ),
      m_Elevator.ChangeStateCommand( Elevator::ElevatorCoralIntake ),
      m_AlgaeIntake.ChangeStateCommand( AlgaeIntake::AngleUp )
    ))); // <- This example method returns CommandPtr

  pathplanner::NamedCommands::registerCommand("AllDown", std::move(frc2::cmd::Parallel(
      m_CoralIntake.ChangeStateCommand( CoralIntake::AngleUp, CoralIntake::intakeStop ),
      m_Elevator.ChangeStateCommand( Elevator::ElevatorStartingConfig ),
      m_AlgaeIntake.ChangeStateCommand( AlgaeIntake::AngleUp )
    ))); // <- This example method returns CommandPtr

  pathplanner::NamedCommands::registerCommand("CoralIntakeFwd", std::move(frc2::cmd::Parallel(
      m_CoralIntake.ChangeStateCommand( CoralIntake::AngleMaintain, CoralIntake::intakeIntake ),
      m_Elevator.ChangeStateCommand( Elevator::ElevatorMaintain ),
      m_AlgaeIntake.ChangeStateCommand( AlgaeIntake::AngleMaintain )
    ))); // <- This example method returns CommandPtr

    pathplanner::NamedCommands::registerCommand("CoralIntakeRev", std::move(frc2::cmd::Parallel(
      m_CoralIntake.ChangeStateCommand( CoralIntake::AngleMaintain, CoralIntake::intakeReverse ),
      m_Elevator.ChangeStateCommand( Elevator::ElevatorMaintain ),
      m_AlgaeIntake.ChangeStateCommand( AlgaeIntake::AngleMaintain )
    )));

  pathplanner::NamedCommands::registerCommand("CoralIntakeStop", std::move(frc2::cmd::Parallel(
      m_CoralIntake.ChangeStateCommand( CoralIntake::AngleMaintain, CoralIntake::intakeStop ),
      m_Elevator.ChangeStateCommand( Elevator::ElevatorMaintain ),
      m_AlgaeIntake.ChangeStateCommand( AlgaeIntake::AngleMaintain)
    ))); // <- This example method returns CommandPtr

  pathplanner::NamedCommands::registerCommand("SetStartConfig", std::move(frc2::cmd::Parallel(
      m_CoralIntake.ChangeStateCommand( CoralIntake::AngleUp, CoralIntake::intakeStop ),
      m_Elevator.ChangeStateCommand( Elevator::ElevatorStartingConfig ),
      m_AlgaeIntake.ChangeStateCommand( AlgaeIntake::AngleUp)
    ))); // <- This example method returns CommandPtr

  //****************************************************************************
  autoChooser = pathplanner::AutoBuilder::buildAutoChooser();
  frc::SmartDashboard::PutData("Auto Mode", &autoChooser);

  // Configure the button bindings
  ConfigureBindings();

#if UseElasticNetTable
  ContainerNetTable = ContainerNetInst.GetTable("2227 RobotContainer");
  ContainerNetInst.StartServer();
#endif

#if UseShuffleboardAPI
  PlaceSmartdashboard();
#endif

}




void RobotContainer::ConfigureBindings() {



  // Configure your trigger bindings here
  m_Drivetrain.SetDefaultCommand(frc2::RunCommand(
      [this] {
        #if 0
        auto xspeed = - m_yspeedLimiter.Calculate(m_driveController.GetLeftY()  * m_Drivetrain.GetMaxSpeed();
        auto yspeed = - m_xspeedLimiter.Calculate(m_driveController.GetLeftX() * m_Drivetrain.GetMaxSpeed();
        #else
        auto xspeed = - m_yspeedLimiter.Calculate(m_driveController.GetLeftY())  * m_Drivetrain.GetMaxSpeed();
        auto yspeed = - m_xspeedLimiter.Calculate(m_driveController.GetLeftX())  * m_Drivetrain.GetMaxSpeed();
        #endif
        auto rotspeed = - m_rotLimiter.Calculate(m_driveController.GetRightX()) * Drivetrain::kMaxAngularSpeed;
        frc::ChassisSpeeds sendChassisSpeed = frc::ChassisSpeeds {xspeed, yspeed, rotspeed};
        m_Drivetrain.drive(
            // Multiply by max speed to map the joystick unitless inputs to
            // actual units. This will map the [-1, 1] to [max speed backwards,
            // max speed forwards], converting them to actual units.
            sendChassisSpeed,
            Constants::kJoystickFieldRelative, frc::TimedRobot::kDefaultPeriod);
      }, 
      {&m_Drivetrain}));

  m_CoralIntake.SetDefaultCommand(frc2::RunCommand(
      [this] {
        m_CoralIntake.ChangeState( CoralIntake::AngleMaintain, CoralIntake::intakeMaintain );
      },
      {&m_CoralIntake}));

  m_AlgaeIntake.SetDefaultCommand(frc2::RunCommand(
      [this] {
        m_AlgaeIntake.ChangeState( AlgaeIntake::AngleMaintain );
      },
      {&m_AlgaeIntake}));

  m_Elevator.SetDefaultCommand(frc2::RunCommand(
      [this] {
        m_Elevator.ChangeState( Elevator::ElevatorMaintain );
      },
      {&m_Elevator}));

  m_Climber.SetDefaultCommand(frc2::RunCommand(
      [this] {
        m_Climber.ChangeState( Climber::ClimberWinchMaintain, Climber::GrabMaintain );
      },
      {&m_Climber}));



  // Adjust delay
  m_buttons.Button(2).OnTrue( 
    frc2::RunCommand(
      [this] {
        if ( delayTime < 60.0_s )
        {
          delayTime += 5.0_s;
        }
      },
      {}
    ).ToPtr()
  );

  m_buttons.Button(5).OnTrue( 
    frc2::RunCommand(
      [this] {
        if ( delayTime > 8.0_s )
        {
          delayTime -= 5.0_s;
        }
      },
      {}
    ).ToPtr()
  );



  // Cycle through things - cyc
  m_buttons.Button(1).ToggleOnTrue( 
    frc2::cmd::RepeatingSequence(
      // place
      frc2::cmd::Parallel(
        m_CoralIntake.ChangeStateCommand( CoralIntake::AnglePlaceCoral, CoralIntake::intakeStop ),
        m_Elevator.ChangeStateCommand( Elevator::ElevatorCoralL3 ),
        m_AlgaeIntake.ChangeStateCommand( AlgaeIntake::AngleUp)
      ),
      frc2::cmd::Wait(3.0_s),
      frc2::cmd::Parallel(
        m_CoralIntake.ChangeStateCommand( CoralIntake::AnglePlaceCoral, CoralIntake::intakeReverse ),
        m_Elevator.ChangeStateCommand( Elevator::ElevatorCoralL3 ),
        m_AlgaeIntake.ChangeStateCommand( AlgaeIntake::AngleUp )
      ),
      frc2::cmd::Wait(1.0_s),
      frc2::cmd::Parallel(
        m_CoralIntake.ChangeStateCommand( CoralIntake::AngleUp, CoralIntake::intakeStop ),
        m_Elevator.ChangeStateCommand( Elevator::ElevatorStartingConfig ),
        m_AlgaeIntake.ChangeStateCommand( AlgaeIntake::AngleUp)
      ),
      //Down and wait
      frc2::cmd::Parallel(
        m_CoralIntake.ChangeStateCommand( CoralIntake::AngleUp, CoralIntake::intakeStop ),
        m_Elevator.ChangeStateCommand( Elevator::ElevatorStartingConfig ),
        m_AlgaeIntake.ChangeStateCommand( AlgaeIntake::AngleUp)
      ),
      WaitCommand2(&delayTime).ToPtr(),
      // Shake
      frc2::cmd::Parallel(
        m_CoralIntake.ChangeStateCommand( CoralIntake::AnglePlaceCoral, CoralIntake::intakeStop ),
        m_Elevator.ChangeStateCommand( Elevator::ElevatorCoralL4 ),
        m_AlgaeIntake.ChangeStateCommand( AlgaeIntake::AngleUp)
      ),
      frc2::cmd::Wait(1.0_s),
      frc2::cmd::Parallel(
        m_CoralIntake.ChangeStateCommand( CoralIntake::AnglePlaceCoral, CoralIntake::intakeStop ),
        m_Elevator.ChangeStateCommand( Elevator::ElevatorCoralL4 ),
        m_AlgaeIntake.ChangeStateCommand( AlgaeIntake::AngleDn )
      ),
      frc2::cmd::Wait(1.0_s),
      frc2::cmd::Parallel(
        m_CoralIntake.ChangeStateCommand( CoralIntake::AnglePlaceCoral, CoralIntake::intakeStop ),
        m_Elevator.ChangeStateCommand( Elevator::ElevatorCoralL4 ),
        m_AlgaeIntake.ChangeStateCommand( AlgaeIntake::AngleUp)
      ),
      frc2::cmd::Wait(1.0_s),
      frc2::cmd::Parallel(
        m_CoralIntake.ChangeStateCommand( CoralIntake::AnglePlaceCoral, CoralIntake::intakeStop ),
        m_Elevator.ChangeStateCommand( Elevator::ElevatorCoralL4 ),
        m_AlgaeIntake.ChangeStateCommand( AlgaeIntake::AngleDn )
      ),
      frc2::cmd::Wait(1.0_s),
      frc2::cmd::Parallel(
        m_CoralIntake.ChangeStateCommand( CoralIntake::AnglePlaceCoral, CoralIntake::intakeStop ),
        m_Elevator.ChangeStateCommand( Elevator::ElevatorCoralL4 ),
        m_AlgaeIntake.ChangeStateCommand( AlgaeIntake::AngleUp)
      ),
      frc2::cmd::Wait(1.0_s),
      frc2::cmd::Parallel(
        m_CoralIntake.ChangeStateCommand( CoralIntake::AnglePlaceCoral, CoralIntake::intakeStop ),
        m_Elevator.ChangeStateCommand( Elevator::ElevatorCoralL4 ),
        m_AlgaeIntake.ChangeStateCommand( AlgaeIntake::AngleDn )
      ),
      frc2::cmd::Wait(1.0_s),
      frc2::cmd::Parallel(
        m_CoralIntake.ChangeStateCommand( CoralIntake::AnglePlaceCoral, CoralIntake::intakeStop ),
        m_Elevator.ChangeStateCommand( Elevator::ElevatorCoralL4 ),
        m_AlgaeIntake.ChangeStateCommand( AlgaeIntake::AngleUp)
      ),
      frc2::cmd::Wait(1.0_s),
      frc2::cmd::Parallel(
        m_CoralIntake.ChangeStateCommand( CoralIntake::AnglePlaceCoral, CoralIntake::intakeStop ),
        m_Elevator.ChangeStateCommand( Elevator::ElevatorCoralL4 ),
        m_AlgaeIntake.ChangeStateCommand( AlgaeIntake::AngleDn )
      ),
      //Down and wait
      frc2::cmd::Parallel(
        m_CoralIntake.ChangeStateCommand( CoralIntake::AngleUp, CoralIntake::intakeStop ),
        m_Elevator.ChangeStateCommand( Elevator::ElevatorStartingConfig ),
        m_AlgaeIntake.ChangeStateCommand( AlgaeIntake::AngleUp)
      ),
      WaitCommand2(&delayTime).ToPtr()
    )
  );




  // Climber Spin - grab
  m_buttons.Button(7).ToggleOnTrue( 
    frc2::cmd::RepeatingSequence(
      m_Climber.ChangeStateCommand( Climber::ClimberWinchStop, Climber::GrabVertical ),
      frc2::cmd::Wait(0.8_s),
      m_Climber.ChangeStateCommand( Climber::ClimberWinchStop, Climber::GrabHorizontal ),
      frc2::cmd::Wait(5.5_s)
    )
  );

  // Elevator up/outtake - plc
  m_buttons.Button(4).ToggleOnTrue( 
    frc2::cmd::RepeatingSequence(
      frc2::cmd::Parallel(
        m_CoralIntake.ChangeStateCommand( CoralIntake::AnglePlaceCoral, CoralIntake::intakeStop ),
        m_Elevator.ChangeStateCommand( Elevator::ElevatorCoralL3 ),
        m_AlgaeIntake.ChangeStateCommand( AlgaeIntake::AngleUp)
      ),
      frc2::cmd::Wait(3.0_s),
      frc2::cmd::Parallel(
        m_CoralIntake.ChangeStateCommand( CoralIntake::AnglePlaceCoral, CoralIntake::intakeReverse ),
        m_Elevator.ChangeStateCommand( Elevator::ElevatorCoralL3 ),
        m_AlgaeIntake.ChangeStateCommand( AlgaeIntake::AngleUp )
      ),
      frc2::cmd::Wait(1.0_s),
      frc2::cmd::Parallel(
        m_CoralIntake.ChangeStateCommand( CoralIntake::AngleUp, CoralIntake::intakeStop ),
        m_Elevator.ChangeStateCommand( Elevator::ElevatorStartingConfig ),
        m_AlgaeIntake.ChangeStateCommand( AlgaeIntake::AngleUp)
      ),
      frc2::cmd::Wait(delayTime)
    )
  );


  // Elevator up/shake - shk
  m_buttons.Button(6).ToggleOnTrue( 
    frc2::cmd::RepeatingSequence(
      frc2::cmd::Parallel(
        m_CoralIntake.ChangeStateCommand( CoralIntake::AnglePlaceCoral, CoralIntake::intakeStop ),
        m_Elevator.ChangeStateCommand( Elevator::ElevatorCoralL4 ),
        m_AlgaeIntake.ChangeStateCommand( AlgaeIntake::AngleUp)
      ),
      frc2::cmd::Wait(1.0_s),
      frc2::cmd::Parallel(
        m_CoralIntake.ChangeStateCommand( CoralIntake::AnglePlaceCoral, CoralIntake::intakeStop ),
        m_Elevator.ChangeStateCommand( Elevator::ElevatorCoralL4 ),
        m_AlgaeIntake.ChangeStateCommand( AlgaeIntake::AngleDn )
      ),
      frc2::cmd::Wait(1.0_s)
    )
  );


  // Hold
  m_buttons.Button(3).WhileTrue( frc2::cmd::Parallel(
      m_CoralIntake.ChangeStateCommand( CoralIntake::AngleMaintain, CoralIntake::intakeStop ),
      m_Elevator.ChangeStateCommand( Elevator::ElevatorMaintain ),
      m_AlgaeIntake.ChangeStateCommand( AlgaeIntake::AngleMaintain),
      m_Climber.ChangeStateCommand( Climber::ClimberWinchStop, Climber::GrabMaintain )
    )
  );

  // All Down / stop
  m_buttons.Button(0).WhileTrue( frc2::cmd::Parallel(
      m_CoralIntake.ChangeStateCommand( CoralIntake::AngleUp, CoralIntake::intakeStop ),
      m_Elevator.ChangeStateCommand( Elevator::ElevatorStartingConfig ),
      m_AlgaeIntake.ChangeStateCommand( AlgaeIntake::AngleUp),
      m_Climber.ChangeStateCommand( Climber::ClimberWinchStop, Climber::GrabStop )
    )
  );

#if 0

  // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
  // frc2::Trigger([this] {
  //   return m_subsystem.ExampleCondition();
  // }).WhileTrue(ExampleCommand(&m_subsystem).ToPtr());

  // Schedule `ExampleMethodCommand` when the Xbox controller's B button is
  // pressed, cancelling on release.
  //m_driverController.B().WhileTrue(m_subsystem.ExampleMethodCommand());
  frc2::Trigger DriverCoralOuttake([this]{return m_driveController.GetRightTriggerAxis() < 0.4;});

  m_Drivetrain.SetDefaultCommand(frc2::RunCommand(
      [this] {
        #if 0
        auto xspeed = - m_yspeedLimiter.Calculate(m_driveController.GetLeftY()  * m_Drivetrain.GetMaxSpeed();
        auto yspeed = - m_xspeedLimiter.Calculate(m_driveController.GetLeftX() * m_Drivetrain.GetMaxSpeed();
        #else
        auto xspeed = - m_yspeedLimiter.Calculate(m_driveController.GetLeftY())  * m_Drivetrain.GetMaxSpeed();
        auto yspeed = - m_xspeedLimiter.Calculate(m_driveController.GetLeftX())  * m_Drivetrain.GetMaxSpeed();
        #endif
        auto rotspeed = - m_rotLimiter.Calculate(m_driveController.GetRightX()) * Drivetrain::kMaxAngularSpeed;
        frc::ChassisSpeeds sendChassisSpeed = frc::ChassisSpeeds {xspeed, yspeed, rotspeed};
        m_Drivetrain.drive(
            // Multiply by max speed to map the joystick unitless inputs to
            // actual units. This will map the [-1, 1] to [max speed backwards,
            // max speed forwards], converting them to actual units.
            sendChassisSpeed,
            Constants::kJoystickFieldRelative, frc::TimedRobot::kDefaultPeriod);
      }, 
      {&m_Drivetrain}));

  m_CoralIntake.SetDefaultCommand(frc2::RunCommand(
      [this] {
        m_CoralIntake.ChangeState( CoralIntake::AngleMaintain, CoralIntake::intakeMaintain );
      },
      {&m_CoralIntake}));

  m_AlgaeIntake.SetDefaultCommand(frc2::RunCommand(
      [this] {
        m_AlgaeIntake.ChangeState( AlgaeIntake::AngleMaintain );
      },
      {&m_AlgaeIntake}));

  m_Elevator.SetDefaultCommand(frc2::RunCommand(
      [this] {
        m_Elevator.ChangeState( Elevator::ElevatorMaintain );
      },
      {&m_Elevator}));

  m_Climber.SetDefaultCommand(frc2::RunCommand(
      [this] {
        m_Climber.ChangeState( Climber::ClimberWinchMaintain, Climber::GrabMaintain );
      },
      {&m_Climber}));

  //resetIMU
  m_driveController.POVUp().WhileTrue( 
    frc2::cmd::RunOnce([ this ] { m_Drivetrain.ResetIMU(); })
  );

  //all down
  m_buttons.Button(10).WhileTrue( frc2::cmd::Parallel(
      m_CoralIntake.ChangeStateCommand( CoralIntake::AngleUp, CoralIntake::intakeStop ),
      m_Elevator.ChangeStateCommand( Elevator::ElevatorStartingConfig ),
      m_AlgaeIntake.ChangeStateCommand( AlgaeIntake::AngleUp)
    )
  );
  //all down drive controller backup
  m_driveController.A().WhileTrue( frc2::cmd::Parallel(
      m_CoralIntake.ChangeStateCommand( CoralIntake::AngleUp, CoralIntake::intakeStop ),
      m_Elevator.ChangeStateCommand( Elevator::ElevatorStartingConfig ),
      m_AlgaeIntake.ChangeStateCommand( AlgaeIntake::AngleUp)
    )
  );
  //Coral Intake level
  m_buttons.Button(5).WhileTrue( frc2::cmd::Parallel(
      m_CoralIntake.ChangeStateCommand( CoralIntake::AngleDn, CoralIntake::autoIntakeFwd ),
      m_Elevator.ChangeStateCommand( Elevator::ElevatorCoralIntake ),
      m_AlgaeIntake.ChangeStateCommand( AlgaeIntake::AngleUp)
    )
  );

  //Coral L1
  m_buttons.Button(1).WhileTrue( frc2::cmd::Parallel(
      m_CoralIntake.ChangeStateCommand( CoralIntake::AngleDn, CoralIntake::intakeStop ),
      m_Elevator.ChangeStateCommand( Elevator::ElevatorCoralL1 ),
      m_AlgaeIntake.ChangeStateCommand( AlgaeIntake::AngleUp)
    )
  );

  //Coral L2
  m_buttons.Button(2).WhileTrue( frc2::cmd::Parallel(
      m_CoralIntake.ChangeStateCommand( CoralIntake::AnglePlaceCoral, CoralIntake::intakeStop ),
      m_Elevator.ChangeStateCommand( Elevator::ElevatorCoralL2 ),
      m_AlgaeIntake.ChangeStateCommand( AlgaeIntake::AngleUp)
    )
  );

  //Coral L3
  m_buttons.Button(3).WhileTrue( frc2::cmd::Parallel(
      m_CoralIntake.ChangeStateCommand( CoralIntake::AnglePlaceCoral, CoralIntake::intakeStop ),
      m_Elevator.ChangeStateCommand( Elevator::ElevatorCoralL3 ),
      m_AlgaeIntake.ChangeStateCommand( AlgaeIntake::AngleUp)
    )
  );

  //Coral L4
  m_buttons.Button(4).WhileTrue( frc2::cmd::Parallel(
      m_CoralIntake.ChangeStateCommand( CoralIntake::AnglePlaceTopCoral, CoralIntake::intakeStop ),
      m_Elevator.ChangeStateCommand( Elevator::ElevatorCoralL4 ),
      m_AlgaeIntake.ChangeStateCommand( AlgaeIntake::AngleUp)
    )
  );


  //Coral Intake
  ( DriverCoralOuttake && m_buttons.Button(7) ).WhileTrue( frc2::cmd::Parallel(
      m_CoralIntake.ChangeStateCommand( CoralIntake::AngleMaintain, CoralIntake::intakeIntake ),
      m_Elevator.ChangeStateCommand( Elevator::ElevatorMaintain ),
      m_AlgaeIntake.ChangeStateCommand( AlgaeIntake::AngleUp)
    )
  );

  m_buttons.Button(7).OnFalse( frc2::cmd::Parallel(
      m_CoralIntake.ChangeStateCommand( CoralIntake::AngleMaintain, CoralIntake::intakeStop ),
      m_Elevator.ChangeStateCommand( Elevator::ElevatorMaintain ),
      m_AlgaeIntake.ChangeStateCommand( AlgaeIntake::AngleUp)
    )
  );

  //Coral reverse
  ( DriverCoralOuttake && m_buttons.Button(8) ).WhileTrue( frc2::cmd::Parallel(
      m_CoralIntake.ChangeStateCommand( CoralIntake::AngleMaintain, CoralIntake::intakeReverse ),
      m_Elevator.ChangeStateCommand( Elevator::ElevatorMaintain ),
      m_AlgaeIntake.ChangeStateCommand( AlgaeIntake::AngleUp)
    )
  );

  m_buttons.Button(8).OnFalse( frc2::cmd::Parallel(
      m_CoralIntake.ChangeStateCommand( CoralIntake::AngleMaintain, CoralIntake::intakeStop ),
      m_Elevator.ChangeStateCommand( Elevator::ElevatorMaintain ),
      m_AlgaeIntake.ChangeStateCommand( AlgaeIntake::AngleUp)
    )
  );



  //climber out
  m_buttons.Button(9).WhileTrue( frc2::cmd::Parallel(
      m_CoralIntake.ChangeStateCommand( CoralIntake::AngleVertical, CoralIntake::intakeStop ),//AngleUp
      m_Elevator.ChangeStateCommand( Elevator::ElevatorStartingConfig ),
      m_Climber.ChangeStateCommand( Climber::ClimberWinchOutManual, Climber::GrabVertical ),
      m_AlgaeIntake.ChangeStateCommand( AlgaeIntake::AngleUp)
    )
  );

  m_buttons.Button(9).OnFalse( frc2::cmd::Parallel(
      m_CoralIntake.ChangeStateCommand( CoralIntake::AngleVertical, CoralIntake::intakeStop ),
      m_Elevator.ChangeStateCommand( Elevator::ElevatorStartingConfig ),
      m_Climber.ChangeStateCommand( Climber::ClimberWinchStop, Climber::GrabVertical ),
      m_AlgaeIntake.ChangeStateCommand( AlgaeIntake::AngleUp)
    )
  );

  //climber in
  m_buttons.Button(6).WhileTrue( frc2::cmd::Parallel(
      m_CoralIntake.ChangeStateCommand( CoralIntake::AngleVertical, CoralIntake::intakeStop ),
      m_Elevator.ChangeStateCommand( Elevator::ElevatorStartingConfig ),
      m_Climber.ChangeStateCommand( Climber::ClimberWinchInManual, Climber::GrabHorizontal ),
      m_AlgaeIntake.ChangeStateCommand( AlgaeIntake::AngleUp)
    )
  );

  m_buttons.Button(6).OnFalse( frc2::cmd::Parallel(
      m_CoralIntake.ChangeStateCommand( CoralIntake::AngleVertical, CoralIntake::intakeStop ),
      m_Elevator.ChangeStateCommand( Elevator::ElevatorStartingConfig ),
      m_Climber.ChangeStateCommand( Climber::ClimberWinchHold, Climber::GrabHorizontal ),
      m_AlgaeIntake.ChangeStateCommand( AlgaeIntake::AngleUp)
    )
  );


  //****************** Algae Bumping ***********************
  m_driveController.LeftBumper().WhileTrue( frc2::cmd::Parallel( //L2 Bump prep
    m_Elevator.ChangeStateCommand(Elevator::ElevatorL2PrepBump),
    m_CoralIntake.ChangeStateCommand( CoralIntake::AngleVertical, CoralIntake::intakeStop ),
    m_AlgaeIntake.ChangeStateCommand( AlgaeIntake::AngleDn)
  ));

 m_driveController.LeftBumper().OnFalse( frc2::cmd::Parallel( //L2 Bump top
    m_Elevator.ChangeStateCommand(Elevator::ElevatorL2EndBump),
    m_CoralIntake.ChangeStateCommand( CoralIntake::AngleVertical, CoralIntake::intakeStop ),
    m_AlgaeIntake.ChangeStateCommand( AlgaeIntake::AngleDn)
  ));

 m_driveController.RightBumper().WhileTrue( frc2::cmd::Parallel( //L3 Bump prep
    m_Elevator.ChangeStateCommand(Elevator::ElevatorL3PrepBump),
    m_CoralIntake.ChangeStateCommand( CoralIntake::AngleVertical, CoralIntake::intakeStop ),
    m_AlgaeIntake.ChangeStateCommand( AlgaeIntake::AngleDn)
  ));

 m_driveController.RightBumper().OnFalse( frc2::cmd::Parallel( //L3 Bump top
    m_Elevator.ChangeStateCommand(Elevator::ElevatorL3EndBump),
    m_CoralIntake.ChangeStateCommand( CoralIntake::AngleVertical, CoralIntake::intakeStop ),
    m_AlgaeIntake.ChangeStateCommand( AlgaeIntake::AngleDn)
  ));

#endif

}

frc2::Command *RobotContainer::GetAutonomousCommand()
{
  return autoChooser.GetSelected();
  // An example command will be run in autonomous
  //return autos::ExampleAuto(&m_Drivetrain);
}




void RobotContainer::UpdateSmartDashboardData() {
  m_Climber.UpdateSmartDashboardData();
  m_Drivetrain.UpdateSmartDashboardData();
  m_Elevator.UpdateSmartDashboardData();
  m_CoralIntake.UpdateSmartDashboardData();
  m_AlgaeIntake.UpdateSmartDashboardData();

  double totalCurrent = ZipZap.GetTotalCurrent();
  double robotVoltage = ZipZap.GetVoltage();
  ContainerNetTable->PutNumber("Total Current", totalCurrent);
  ContainerNetTable->PutNumber("Robot Voltage", robotVoltage);


  double matchTimeSeconds = frc::DriverStation::GetMatchTime().value(); // Convert to double
  // Convert match time to mm:ss format
  int minutes = static_cast<int>(matchTimeSeconds) / 60;
  int seconds = static_cast<int>(matchTimeSeconds) % 60;
  std::ostringstream timeStream;
  timeStream << std::setw(2) << std::setfill('0') << minutes << ":"
              << std::setw(2) << std::setfill('0') << seconds;
 
  #if UseElasticNetTable
  ContainerNetTable->PutString("Match_Time_Str",timeStream.str());
  ContainerNetTable->PutNumber("Match_Time",frc::DriverStation::GetMatchTime().value());
  ContainerNetTable->PutString("Alliance",DetermineAlliance());
  #endif

  #if UseShuffleboardAPI
  m_AllianceDisp->SetString(DetermineAlliance());
  // Update the Shuffleboard entry
  m_matchTime->SetString(timeStream.str());
  #endif

}


void RobotContainer::TeleopInit() {
  m_Climber.TeleopInit();
  m_Drivetrain.TeleopInit();
  m_Elevator.TeleopInit();
  m_CoralIntake.TeleopInit();
}

void RobotContainer::AutonomousInit()
{
  m_Drivetrain.AutonomousInit();
    m_CoralIntake.ChangeState( CoralIntake::AngleUp, CoralIntake::intakeStop );
    m_Elevator.ChangeState( Elevator::ElevatorStartingConfig );
    m_AlgaeIntake.ChangeState( AlgaeIntake::AngleUp);
} 

#if UseShuffleboardAPI

void RobotContainer::PlaceSmartdashboard() {

    frc::Shuffleboard::GetTab(Constants::kDriverTabName)
    .Add("Autonomous Mode", autoChooser)
    .WithSize(2, 1)
    .WithPosition(0, 0);
}

#endif

std::string RobotContainer::DetermineAlliance()
{
  std::optional<frc::DriverStation::Alliance> alliance = frc::DriverStation::GetAlliance();
  std::optional<int> position = frc::DriverStation::GetLocation();
  std::string allianceColor = "Unknown"; // Default value
  if (alliance.has_value()) {
      switch (alliance.value()) {
        case frc::DriverStation::Alliance::kRed:
          allianceColor = "Red";
          break;
        case frc::DriverStation::Alliance::kBlue:
          allianceColor = "Blue";
          break;
      }
    }
  std::string positionStr;
  if (position) {
          // Use std::stringstream to convert the int to a string
          std::stringstream ss;
          ss << *position;  // Dereference the std::optional
          positionStr = ss.str();
      } else {
          positionStr = "Unknown";
      }

    std::string allianceDisplay = "Alliance: " + allianceColor + " --- Position: " + positionStr;
    return allianceDisplay;
}