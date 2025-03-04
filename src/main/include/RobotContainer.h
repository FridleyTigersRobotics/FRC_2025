// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc2/command/button/CommandGenericHID.h>
#include <frc/filter/SlewRateLimiter.h>

#include "Constants.h"
#include "subsystems/Climber.h"
#include "subsystems/Drivetrain.h"
#include "subsystems/Elevator.h"
#include "subsystems/CoralIntake.h"
#include "subsystems/AlgaeIntake.h"

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();
  void UpdateSmartDashboardData();
  void TeleopInit();
  void AutonomousInit();

 private:
  // Replace with CommandPS4Controller or CommandJoystick if needed
  // frc2::CommandXboxController m_driverController{
  //     OperatorConstants::kDriverControllerPort};

  frc2::CommandXboxController m_driveController{0};
  frc2::CommandGenericHID     m_buttons        {1};

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  frc::SlewRateLimiter<units::scalar> m_xspeedLimiter{3 / 1_s};
  frc::SlewRateLimiter<units::scalar> m_yspeedLimiter{3 / 1_s};
  frc::SlewRateLimiter<units::scalar> m_rotLimiter{3 / 1_s};

  // The robot's subsystems are defined here...


  Climber     m_Climber;
  Drivetrain  m_Drivetrain;
  Elevator    m_Elevator;
  CoralIntake m_CoralIntake{ m_Elevator };
  AlgaeIntake m_AlgaeIntake{ m_Elevator };

  void ConfigureBindings();

  /* Path follower */
  frc::SendableChooser<frc2::Command *> autoChooser;
  public:
  frc2::Command *GetAutonomousCommand();

};
