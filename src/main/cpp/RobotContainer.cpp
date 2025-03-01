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

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here

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
        auto xspeed = m_yspeedLimiter.Calculate(m_driveController.GetLeftY())  * Drivetrain::kMaxSpeed;
        auto yspeed = m_xspeedLimiter.Calculate(m_driveController.GetLeftX())  * Drivetrain::kMaxSpeed;
        auto rotspeed = m_rotLimiter.Calculate(m_driveController.GetRightX()) * Drivetrain::kMaxAngularSpeed;
        frc::ChassisSpeeds sendChassisSpeed = frc::ChassisSpeeds {xspeed, yspeed, rotspeed};
        m_Drivetrain.drive(
            // Multiply by max speed to map the joystick unitless inputs to
            // actual units. This will map the [-1, 1] to [max speed backwards,
            // max speed forwards], converting them to actual units.
            sendChassisSpeed,
            Constants::kJoystickFieldRelative, frc::TimedRobot::kDefaultPeriod);
      },
      {&m_Drivetrain}));

  m_Intake.SetDefaultCommand(frc2::RunCommand(
      [this] {
        m_Intake.ChangeStateCommand( Intake::AngleMaintain, Intake::intakeStop );
      },
      {&m_Intake}));

  m_Elevator.SetDefaultCommand(frc2::RunCommand(
      [this] {
        m_Elevator.ChangeStateCommand( Elevator::ElevatorMaintain );
      },
      {&m_Elevator}));

  m_Climber.SetDefaultCommand(frc2::RunCommand(
      [this] {
        m_Climber.ChangeStateCommand( Climber::ClimberWinchStop, Climber::GrabMaintain );
      },
      {&m_Climber}));



  //all down
  m_buttons.Button(10).WhileTrue( frc2::cmd::Parallel(
      m_Intake.ChangeStateCommand( Intake::AngleUp, Intake::intakeStop ),
      m_Elevator.ChangeStateCommand( Elevator::ElevatorStartingConfig )
    )
  );

  //Coral Intake level
  m_buttons.Button(5).WhileTrue( frc2::cmd::Parallel(
      m_Intake.ChangeStateCommand( Intake::AnglePlaceCoral, Intake::intakeStop ),
      m_Elevator.ChangeStateCommand( Elevator::ElevatorCoralIntake )
    )
  );

  //Coral L1
  m_buttons.Button(1).WhileTrue( frc2::cmd::Parallel(
      m_Intake.ChangeStateCommand( Intake::AnglePlaceCoral, Intake::intakeStop ),
      m_Elevator.ChangeStateCommand( Elevator::ElevatorCoralL1 )
    )
  );

  //Coral L2
  m_buttons.Button(2).WhileTrue( frc2::cmd::Parallel(
      m_Intake.ChangeStateCommand( Intake::AnglePlaceCoral, Intake::intakeStop ),
      m_Elevator.ChangeStateCommand( Elevator::ElevatorCoralL2 )
    )
  );

  //Coral L3
  m_buttons.Button(3).WhileTrue( frc2::cmd::Parallel(
      m_Intake.ChangeStateCommand( Intake::AnglePlaceCoral, Intake::intakeStop ),
      m_Elevator.ChangeStateCommand( Elevator::ElevatorCoralL3 )
    )
  );

  //Coral L4
  m_buttons.Button(4).WhileTrue( frc2::cmd::Parallel(
      m_Intake.ChangeStateCommand( Intake::AnglePlaceTopCoral, Intake::intakeStop ),
      m_Elevator.ChangeStateCommand( Elevator::ElevatorCoralL4 )
    )
  );


  //Coral Intake
  m_buttons.Button(7).WhileTrue( frc2::cmd::Parallel(
      m_Intake.ChangeStateCommand( Intake::AngleMaintain, Intake::intakeIntake ),
      m_Elevator.ChangeStateCommand( Elevator::ElevatorMaintain )
    )
  );

  //Coral reverse
  m_buttons.Button(8).WhileTrue( frc2::cmd::Parallel(
      m_Intake.ChangeStateCommand( Intake::AngleMaintain, Intake::intakeReverse ),
      m_Elevator.ChangeStateCommand( Elevator::ElevatorMaintain )
    )
  );



  //climber out
  m_buttons.Button(9).WhileTrue( frc2::cmd::Parallel(
      m_Intake.ChangeStateCommand( Intake::AngleUp, Intake::intakeStop ),
      m_Elevator.ChangeStateCommand( Elevator::ElevatorStartingConfig ),
      m_Climber.ChangeStateCommand( Climber::ClimberWinchOutManual, Climber::GrabVertical )
    )
  );

  //climber in
  m_buttons.Button(6).WhileTrue( frc2::cmd::Parallel(
      m_Intake.ChangeStateCommand( Intake::AngleUp, Intake::intakeStop ),
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
  m_Intake.UpdateSmartDashboardData();
}


void RobotContainer::TeleopInit() {
  m_Climber.TeleopInit();
  m_Drivetrain.TeleopInit();
  m_Elevator.TeleopInit();
  m_Intake.TeleopInit();
}


