// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc2/command/button/CommandGenericHID.h>


#include "Constants.h"
#include "subsystems/Climber.h"
#include "subsystems/Drivetrain.h"
#include "subsystems/Elevator.h"
#include "subsystems/Intake.h"

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

  frc2::CommandPtr GetAutonomousCommand();

  void UpdateSmartDashboardData();
  void TeleopInit();

 private:
  // Replace with CommandPS4Controller or CommandJoystick if needed
  // frc2::CommandXboxController m_driverController{
  //     OperatorConstants::kDriverControllerPort};

  frc2::CommandXboxController m_driveController{0};
  frc2::CommandGenericHID     m_buttons        {1};

  // The robot's subsystems are defined here...


  Climber    m_Climber;
  Drivetrain m_Drivetrain;
  Elevator   m_Elevator;
  Intake     m_Intake{ m_Elevator };

  void ConfigureBindings();
};
