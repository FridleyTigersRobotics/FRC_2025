// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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


RobotContainer::RobotContainer() : m_Elevator(), m_CoralIntake(m_Elevator) {
  // Initialize all of your commands and subsystems here

  // Register Named Commands. You must pass either a CommandPtr rvalue or a shared_ptr to the command, not the command directly.
  pathplanner::NamedCommands::registerCommand("ElevatorL2", std::move(frc2::cmd::Parallel(
      m_CoralIntake.ChangeStateCommand( CoralIntake::AnglePlaceCoral, CoralIntake::intakeStop ),
      m_Elevator.ChangeStateCommand( Elevator::ElevatorCoralL2 )
    ))); // <- This example method returns CommandPtr

  autoChooser = pathplanner::AutoBuilder::buildAutoChooser();
  frc::SmartDashboard::PutData("Auto Mode", &autoChooser);

  // Configure the button bindings
  ConfigureBindings();


}

void RobotContainer::ConfigureBindings() {
  // Configure your trigger bindings here

  // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
  // frc2::Trigger([this] {
  //   return m_subsystem.ExampleCondition();
  // }).WhileTrue(ExampleCommand(&m_subsystem).ToPtr());

  // Schedule `ExampleMethodCommand` when the Xbox controller's B button is
  // pressed, cancelling on release.
  //m_driverController.B().WhileTrue(m_subsystem.ExampleMethodCommand());


  m_Drivetrain.SetDefaultCommand(frc2::RunCommand(
      [this] {
        auto xspeed = - m_yspeedLimiter.Calculate(m_driveController.GetLeftY())  * Drivetrain::kMaxSpeed;
        auto yspeed = - m_xspeedLimiter.Calculate(m_driveController.GetLeftX())  * Drivetrain::kMaxSpeed;
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
        m_AlgaeIntake.ChangeState( AlgaeIntake::AngleMaintain, AlgaeIntake::intakeMaintain );
      },
      {&m_AlgaeIntake}));

  m_Elevator.SetDefaultCommand(frc2::RunCommand(
      [this] {
        m_Elevator.ChangeState( Elevator::ElevatorMaintain );
      },
      {&m_Elevator}));

  m_Climber.SetDefaultCommand(frc2::RunCommand(
      [this] {
        m_Climber.ChangeState( Climber::ClimberWinchStop, Climber::GrabMaintain );
      },
      {&m_Climber}));

  //resetIMU
  m_driveController.POVUp().WhileTrue(
    frc2::cmd::RunOnce([ this ] { m_Drivetrain.ResetIMU(); })
  );

  //all down
  m_buttons.Button(10).WhileTrue( frc2::cmd::Parallel(
      m_AlgaeIntake.ChangeStateCommand( AlgaeIntake::AngleUp, AlgaeIntake::intakeMaintain ),
      m_Elevator.ChangeStateCommand( Elevator::ElevatorStartingConfig )
    )
  );

  //Algae Intake level
  m_buttons.Button(5).WhileTrue( frc2::cmd::Parallel(
      m_AlgaeIntake.ChangeStateCommand( AlgaeIntake::AngleDn, AlgaeIntake::intakeMaintain ),
      m_Elevator.ChangeStateCommand( Elevator::ElevatorCoralIntake )
    )
  );

  //Algae L1
  m_buttons.Button(1).WhileTrue( frc2::cmd::Parallel(
      m_AlgaeIntake.ChangeStateCommand( AlgaeIntake::AngleDn, AlgaeIntake::intakeMaintain ),
      m_Elevator.ChangeStateCommand( Elevator::ElevatorCoralL1 )
    )
  );

  //Algae L2
  m_buttons.Button(2).WhileTrue( frc2::cmd::Parallel(
      m_AlgaeIntake.ChangeStateCommand( AlgaeIntake::AngleDn, AlgaeIntake::intakeMaintain ),
      m_Elevator.ChangeStateCommand( Elevator::ElevatorCoralL2 )
    )
  );



  //Algae Intake
  m_buttons.Button(7).WhileTrue( frc2::cmd::Parallel(
      m_AlgaeIntake.ChangeStateCommand( AlgaeIntake::AngleMaintain, AlgaeIntake::intakeIntake ),
      m_Elevator.ChangeStateCommand( Elevator::ElevatorMaintain )
    )
  );

  m_buttons.Button(7).OnFalse( frc2::cmd::Parallel(
      m_AlgaeIntake.ChangeStateCommand( AlgaeIntake::AngleMaintain, AlgaeIntake::intakeStop ),
      m_Elevator.ChangeStateCommand( Elevator::ElevatorMaintain )
    )
  );

  //Algae reverse
  m_buttons.Button(8).WhileTrue( frc2::cmd::Parallel(
      m_AlgaeIntake.ChangeStateCommand( AlgaeIntake::AngleMaintain, AlgaeIntake::intakeReverse ),
      m_Elevator.ChangeStateCommand( Elevator::ElevatorMaintain )
    )
  );

  m_buttons.Button(8).OnFalse( frc2::cmd::Parallel(
      m_AlgaeIntake.ChangeStateCommand( AlgaeIntake::AngleMaintain, AlgaeIntake::intakeStop ),
      m_Elevator.ChangeStateCommand( Elevator::ElevatorMaintain )
    )
  );



  //climber out
  m_buttons.Button(9).WhileTrue( frc2::cmd::Parallel(
      m_Elevator.ChangeStateCommand( Elevator::ElevatorStartingConfig ),
      m_Climber.ChangeStateCommand( Climber::ClimberWinchOutManual, Climber::GrabVertical )
    )
  );

  //climber in
  m_buttons.Button(6).WhileTrue( frc2::cmd::Parallel(
      m_Elevator.ChangeStateCommand( Elevator::ElevatorStartingConfig ),
      m_Climber.ChangeStateCommand( Climber::ClimberWinchInManual, Climber::GrabHorizontal )
    )
  );



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
}


void RobotContainer::TeleopInit() {
  m_Climber.TeleopInit();
  m_Drivetrain.TeleopInit();
  m_Elevator.TeleopInit();
  m_CoralIntake.TeleopInit();
}


