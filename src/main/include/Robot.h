#pragma once

#include <string>
#include <frc/Timer.h>
#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <Phoenix5.h>
#include <units/angular_velocity.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/acceleration.h>
#include <units/voltage.h>
#include "units/angular_acceleration.h"

// Subsystems
#include <Drivetrain.h>
#include <Climber.h>
#include <Elevator.h>
#include <Claw.h>
#include "LimelightHelpers.h"

class Robot : public frc::TimedRobot {
  public:
    Robot();
    void RobotInit() override;
    void RobotPeriodic() override;

    void TestInit() override;
    void TestPeriodic() override;

    void AutonomousInit() override;
    void AutonomousPeriodic() override;

    void TeleopInit() override;
    void TeleopPeriodic() override;

    void DisabledInit() override;
    void DisabledPeriodic() override;

    void SimulationInit() override;
    void SimulationPeriodic() override;

    private:
      frc::SendableChooser<std::string> m_chooser;
      const std::string kAutoNameDefault = "Default";
      const std::string kAutoNameCustom = "MyAuto";
      std::string m_autoSelected;

      frc::XboxController m_driveController{0};
      frc::GenericHID m_buttons            {1};

      // Subsystems
      Drivetrain m_Drivetrain;
      Climber    m_Climber;
      Elevator   m_Elevator;
      Claw       m_Claw;
      
      bool m_fieldRelative      = true;
};
