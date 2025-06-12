// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#define UseShuffleboardAPI 0
#define UseElasticNetTable 1

#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc2/command/button/CommandGenericHID.h>
#include <frc/filter/SlewRateLimiter.h>
#include <networktables/NetworkTableInstance.h>

#include <frc/PowerDistribution.h>
#include "Constants.h"
#include "subsystems/Climber.h"
#include "subsystems/Drivetrain.h"
#include "subsystems/Elevator.h"
#include "subsystems/CoralIntake.h"
#include "subsystems/AlgaeIntake.h"
#include "subsystems/VisionSubsystem.h"
#include <frc2/command/Command.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>
#include "frc2/command/WaitCommand.h"


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
  std::string DetermineAlliance();

  #if UseShuffleboardAPI
  void PlaceSmartdashboard();
  #endif

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
  Drivetrain  m_Drivetrain{ m_Elevator };
  Elevator    m_Elevator;
  CoralIntake m_CoralIntake{ m_Elevator };
  AlgaeIntake m_AlgaeIntake{ m_Elevator };
  VisionSubsystem m_visionSubsystem;

  void ConfigureBindings();

  /* Path follower */
  frc::SendableChooser<frc2::Command *> autoChooser;
  public:
  frc2::Command *GetAutonomousCommand();
  frc2::WaitCommand WaitBetweenActionsCommand{3.0_s};
  //frc2::CommandPtr WaitBetweenActionsCommandPtr = WaitBetweenActionsCommand.ToPtr();

  private:
  frc::PowerDistribution ZipZap{1, frc::PowerDistribution::ModuleType::kRev};
  units::time::second_t delayTime = 3.5_s;


  #if UseShuffleboardAPI
  nt::GenericEntry *m_AllianceDisp = frc::Shuffleboard::GetTab(Constants::kDriverTabName)
    .Add("Alliance", "Unknown")
    .WithWidget("Text View")
    .WithSize(2, 1)
    .WithPosition(0, 1)
    .GetEntry();

  nt::GenericEntry *m_matchTime = frc::Shuffleboard::GetTab(Constants::kDriverTabName)
    .Add("Match Timer","00.00")
    .WithWidget("Text View")
    .WithSize(1, 1)
    .WithPosition(1, 2)
    .GetEntry();
  #endif

  #if UseElasticNetTable
    nt::NetworkTableInstance ContainerNetInst = nt::NetworkTableInstance::GetDefault();
    std::shared_ptr<nt::NetworkTable> ContainerNetTable;

  #endif

};
