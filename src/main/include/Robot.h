#pragma once

#include <string>
#include <frc/Timer.h>
#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <Phoenix5.h>
#include <Drivetrain.h>

#include <Climber.h>

#include <units/angular_velocity.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/acceleration.h>
#include <units/voltage.h>
#include "units/angular_acceleration.h"


#define NEW_AUTO_CHOOSER ( 0 )
#define DISABLE_SECONDARY_CONTROLLER ( 1 )



class DummyController
{
public:
  bool GetRightBumper(void);
  bool GetLeftBumper(void);
  bool GetAButton(void);
  bool GetBButton(void);
  bool GetXButton(void);
  bool GetYButton(void);
  float GetRightTriggerAxis(void);
  bool GetStartButtonPressed(void);
  bool GetBackButtonPressed(void);
};





class Robot : public frc::TimedRobot {
 public:
    void RobotInit() override;
    void RobotPeriodic() override;

    void TestInit() override;


    void AutonomousInit() override;
    void AutonomousPeriodic() override;

    void TeleopInit() override;
    void TeleopPeriodic() override;

    void DisabledInit() override;


    // Autonomous
    void Drivetrain_Stop();
    void DriveForDistance( 
      units::meter_t                xDistance, 
      units::meter_t                yDistance, 
      units::radian_t               rotRadians, 
      units::meters_per_second_t  xSpeedMult, 
      units::meters_per_second_t  ySpeedMult, 
      units::radians_per_second_t rotSpeed,
      units::time::second_t       maxTime
   );
   void MoveArmForPickup();
   void MoveArmForShooting();
   void AimAndPrepShoot( units::second_t maxTime );
   void Shoot( units::second_t maxTime );
   void Wait( units::second_t maxTime );

    void RunAutoSequence();
    void AutonomousStateInit();
    void AutonomousStateUpdate();
    void AutoAngle(double AngleCrap);

 private:
    frc::XboxController m_driveController{0};
    frc::GenericHID m_buttons            {1};
  #if DISABLE_SECONDARY_CONTROLLER
    DummyController m_coController;
  #else
    frc::XboxController m_coController   {2};
  #endif

    Drivetrain m_Drivetrain;
    
    Climber    m_Climber;
   

    bool m_controlModeEndGame = false; // Switches co-controller to end game mode
    bool m_fieldRelative      = false;

    frc::SlewRateLimiter<units::scalar> m_xspeedLimiter{2 / 1_s};
    frc::SlewRateLimiter<units::scalar> m_yspeedLimiter{2 / 1_s};
    frc::SlewRateLimiter<units::scalar> m_rotLimiter{10 / 1_s};

    frc::Pose2d m_initialPose;


    double m_xyDirP      = 1.000;  
    double m_rotP        = 1.000;  
    units::meters_per_second_t          m_xyMaxVel    { std::numbers::pi * 1_mps };
    units::meters_per_second_squared_t  m_xyMaxAccel  { std::numbers::pi * 1_mps / 1_s };
    units::radians_per_second_t         m_rotMaxVel   { std::numbers::pi * 1_rad_per_s };
    units::radians_per_second_squared_t m_rotMaxAccel { std::numbers::pi * 1_rad_per_s / 1_s };

    units::meter_t             kXyPosTolerance{ 0.025_m };
    units::meters_per_second_t kXyVelTolerance{ 0.05_mps };

    units::radian_t             kRotPosTolerance{ 0.05_rad };
    units::radians_per_second_t kRotVelTolerance{ 0.05_rad_per_s };

    frc::ProfiledPIDController<units::meter> m_AutoXdirPid{
      m_xyDirP,
      0.0,
      0.0,
      {units::meters_per_second_t{m_xyMaxVel}, units::meters_per_second_squared_t{m_xyMaxAccel}}
    };

    frc::ProfiledPIDController<units::meter> m_AutoYdirPid{
      m_xyDirP,
      0.0,
      0.0,
      {units::meters_per_second_t{m_xyMaxVel}, units::meters_per_second_squared_t{m_xyMaxAccel}}
    };

    frc::ProfiledPIDController<units::radians> m_AutoRotatePid{
      1.0,
      0.0,
      0.0,
      {units::radians_per_second_t{m_rotMaxVel}, units::radians_per_second_squared_t{m_rotMaxAccel}}
    };



    frc::PIDController m_LimeRotatePid{
      0.0314159265,
      0.0,
      0.0//,
      //{units::radians_per_second_t{1}, units::radians_per_second_squared_t{1}}
    };

    double m_DriveTargetAngle = 0;
    double m_RotP = 0.05;

    frc::PIDController m_DriveRotatePid{
      m_RotP,
      0.00,
      0.0//,
      //{units::degrees_per_second_t{90.0}, units::degrees_per_second_squared_t{90.0}}
    };

    double m_limeVelMax = 1;
    double m_limeAccMax = 1;    
    double m_limeP      = 1;
    double m_limeI      = 0;
    double m_limeD      = 0;
    double m_limeMaxOutput = 0.6;
    double m_limeMinOutput = 0.20;
    double m_limeMinThresh = 0.02;
    double m_limeAngleOffset = -11;




  // Auto
    std::string TranslateAutoModeToAutoString( uint32_t autoModeInt );
    frc::SendableChooser<std::string> m_autoChooser;
    std::string  m_autoSelected { kAutoNameDefault };
    uint32_t     m_autoSelectedInteger { 0 };
    frc::Timer   m_autoTimer;

    unsigned int m_autoState    { 0 }; 
    bool m_autoStateDone    { false }; 
    bool m_AbortAuto        { false };

    double m_prevAngle       { 0 };
    double m_currentAngle    { 0 };
    double m_angleDelta      { 0 };
    double m_currentAvgAngle { 0 };
    double m_avgAngleDelta   { 0 };
    double m_prevAvgAngle    { 0 };


    const std::string kAutoNameDefault         { "DO NOTHING" };
    const std::string kAutoDrive               { "Drive" };
    const std::string kShootCenter             { "ShootCenter" };
    const std::string kShootCenterPickupCenter { "ShootCenterPickupCenter" };
    const std::string kShootLeftPickupLeft     { "ShootLeftPickupLeft" };
    const std::string kShootRightPickupRight   { "ShootRightPickupRight" };
    const std::string kCenterShootRun          { "CenterShootRun" };
    const std::string kShootRightPickupRightTest   { "ShootRightPickupRightTest" };
    const std::string kShootLeftPickupLeftTest     { "ShootLeftPickupLeftTest" };
    const std::string kShootRunLeft            { "ShootRunLeft" };

  std::vector<std::function<void(void)>> defaultAutoSequence = {
    [this] (void) -> void { Drivetrain_Stop(); },
  };

  // TESTED
  std::vector<std::function<void(void)>> Auto_Drive = {
    [this] (void) -> void { AutoAngle( 0 ); },
    [this] (void) -> void { DriveForDistance( -2.0_m, 0.0_m, 0.0_rad, 0.5_mps, 0.0_mps, 0.0_rad_per_s, 5.0_s ); },
    [this] (void) -> void { Drivetrain_Stop(); },
  };

  // TESTED
  std::vector<std::function<void(void)>> Auto_ShootCenter = {
    //[this] (void) -> void { DriveForDistance( 0.5_m, 0.0_m, 0.0_rad, 0.5_mps, 0.0_mps, 0.0_rad_per_s, 5.0_s ); },
    [this] (void) -> void { AutoAngle( 0 ); },
    [this] (void) -> void { AimAndPrepShoot( 4.0_s ); },
    [this] (void) -> void { Shoot( 1.0_s ); },
 
  };

//TESTED :D :D :D
  std::vector<std::function<void(void)>> Auto_ShootCenterPickupCenter = {
    [this] (void) -> void { AutoAngle( 0 ); },
    //[this] (void) -> void { DriveForDistance( 0.5_m, 0.0_m, 0.0_rad, 0.5_mps, 0.0_mps, 0.0_rad_per_s, 5.0_s ); },
    [this] (void) -> void { AimAndPrepShoot( 4.0_s ); },
    [this] (void) -> void { Shoot( 1.0_s ); },
  
    [this] (void) -> void { MoveArmForPickup(); },
    //[this] (void) -> void { Wait( 1.0_s ); },
    [this] (void) -> void { DriveForDistance( -1.5_m, 0.0_m, 0.0_rad, 1.5_mps, 0.0_mps, 0.0_rad_per_s, 5.0_s ); },
    [this] (void) -> void { MoveArmForShooting(); },
    [this] (void) -> void { DriveForDistance( 1.5_m, 0.0_m, 0.0_rad, 1.5_mps, 0.0_mps, 0.0_rad_per_s, 5.0_s); },
    [this] (void) -> void { AimAndPrepShoot( 4.0_s ); },
    [this] (void) -> void { Shoot( 1.0_s ); },
  
  };


  std::vector<std::function<void(void)>> ShootLeftPickupLeft = {
    //[this] (void) -> void { DriveForDistance( 0.5_m, 0.0_m, 0.0_rad, 0.5_mps, 0.0_mps, 0.0_rad_per_s, 5.0_s ); },
    [this] (void) -> void { AutoAngle( 60); },
    [this] (void) -> void { AimAndPrepShoot( 4.0_s ); },
    [this] (void) -> void { Shoot( 1.0_s ); },
  
    //[this] (void) -> void { Wait( 1.0_s ); },
    [this] (void) -> void { DriveForDistance( -0.7_m, 0.0_m, 0.0_rad, 1.0_mps, 0.0_mps, 0.0_rad_per_s, 5.0_s ); },
    [this] (void) -> void { DriveForDistance( 0.0_m, 0.0_m, 1.15_rad, 0.0_mps, 0.0_mps, 1.0_rad_per_s, 5.0_s ); },
    //[this] (void) -> void { MoveArmForPickup(); },
    [this] (void) -> void { DriveForDistance( -2.00_m, 0.0_m, 0.0_rad, 1.0_mps, 0.0_mps, 0.0_rad_per_s, 5.0_s ); },
    // [this] (void) -> void { DriveForDistance( 0.0_m, -0.1_m, 0.0_rad, 0.0_mps, 0.8_mps, 0.0_rad_per_s, 5.0_s ); },
    // [this] (void) -> void { DriveForDistance( 0.0_m,   0.1_m, 0.0_rad, 0.0_mps, 0.8_mps, 0.0_rad_per_s, 5.0_s ); },

    // Re enable later
    // [this] (void) -> void { MoveArmForShooting(); },
    // [this] (void) -> void { DriveForDistance( 1.35_m, 0.0_m, 0.0_rad, 1.0_mps, 0.0_mps, 0.0_rad_per_s, 5.0_s ); },
    // [this] (void) -> void { DriveForDistance( 0.0_m, 0.0_m, -1.15_rad, 0.0_mps, 0.0_mps, 1.0_rad_per_s, 5.0_s ); },
    // [this] (void) -> void { DriveForDistance( 0.7_m, 0.0_m, 0.0_rad, 1.0_mps, 0.0_mps, 0.0_rad_per_s, 5.0_s ); },
    // [this] (void) -> void { DriveForDistance( 0.0_m, -0.1_m, 0.0_rad, 0.0_mps, 0.5_mps, 0.0_rad_per_s, 5.0_s ); },
    // [this] (void) -> void { AimAndPrepShoot( 2.0_s ); },
    // [this] (void) -> void { Shoot( 1.0_s ); },
  

  };

  std::vector<std::function<void(void)>> ShootRightPickupRight = {
    //[this] (void) -> void { DriveForDistance( 0.5_m, 0.0_m, 0.0_rad, 0.5_mps, 0.0_mps, 0.0_rad_per_s, 5.0_s ); },
    [this] (void) -> void { AutoAngle( -60 ); },
    [this] (void) -> void { AimAndPrepShoot( 4.0_s ); },
    [this] (void) -> void { Shoot( 1.0_s ); },
  
    //[this] (void) -> void { Wait( 1.0_s ); },
    [this] (void) -> void { DriveForDistance( -0.7_m, 0.0_m, 0.0_rad, 1.3_mps, 0.0_mps, 0.0_rad_per_s, 5.0_s ); },
    [this] (void) -> void { DriveForDistance( 0.0_m, 0.0_m, -1.15_rad, 0.0_mps, 0.0_mps, 1.0_rad_per_s, 5.0_s ); },
    [this] (void) -> void { MoveArmForPickup(); },
    [this] (void) -> void { DriveForDistance( -1.35_m, 0.0_m, 0.0_rad, 1.4_mps, 0.0_mps, 0.0_rad_per_s, 5.0_s ); },
    //[this] (void) -> void { DriveForDistance( 0.0_m, -0.1_m, 0.0_rad, 0.0_mps, 0.8_mps, 0.0_rad_per_s, 5.0_s ); },
    //[this] (void) -> void { DriveForDistance( 0.0_m,   0.1_m, 0.0_rad, 0.0_mps, 0.8_mps, 0.0_rad_per_s, 5.0_s ); },
    [this] (void) -> void { MoveArmForShooting(); },
    [this] (void) -> void { DriveForDistance( 1.5_m, 0.0_m, 0.0_rad, 1.4_mps, 0.0_mps, 0.0_rad_per_s, 5.0_s ); },
    [this] (void) -> void { DriveForDistance( 0.0_m, 0.0_m, 1.15_rad, 0.0_mps, 0.0_mps, 1.0_rad_per_s, 5.0_s ); },
    [this] (void) -> void { DriveForDistance( 0.7_m, 0.0_m, 0.0_rad, 1.00_mps, 0.0_mps, 0.0_rad_per_s, 5.0_s ); },
    [this] (void) -> void { DriveForDistance( 0.0_m, -0.1_m, 0.0_rad, 0.0_mps, 0.5_mps, 0.0_rad_per_s, 5.0_s ); },
    [this] (void) -> void { AimAndPrepShoot( 2.0_s ); },
    [this] (void) -> void { Shoot( 1.0_s ); },
  
  };

  std::vector<std::function<void(void)>> Auto_CenterShootRun = {
    //[this] (void) -> void { DriveForDistance( 0.5_m, 0.0_m, 0.0_rad, 0.5_mps, 0.0_mps, 0.0_rad_per_s, 5.0_s ); },
    [this] (void) -> void { AutoAngle( 0 ); },
    [this] (void) -> void { AimAndPrepShoot( 4.0_s ); },
    [this] (void) -> void { Shoot( 1.0_s ); },
   
    [this] (void) -> void { MoveArmForPickup(); },
    //[this] (void) -> void { Wait( 1.0_s ); },
    [this] (void) -> void { DriveForDistance( -1.5_m, 0.0_m, 0.0_rad, 0.9_mps, 0.0_mps, 0.0_rad_per_s, 5.0_s ); },
    [this] (void) -> void { MoveArmForShooting(); },
    [this] (void) -> void { DriveForDistance( 1.5_m, 0.0_m, 0.0_rad, 0.8_mps, 0.0_mps, 0.0_rad_per_s, 5.0_s); },
    [this] (void) -> void { AimAndPrepShoot( 4.0_s ); },
    [this] (void) -> void { Shoot( 1.0_s ); },
  
    [this] (void) -> void {DriveForDistance( -1.5_m, 0.0_m, 0.0_rad, 0.9_mps, 0.0_mps, 0.0_rad_per_s, 5.0_s); },
    [this] (void) -> void { Drivetrain_Stop();},
  };

std::vector<std::function<void(void)>> ShootRightPickupRightTest = {
    //[this] (void) -> void { DriveForDistance( 0.5_m, 0.0_m, 0.0_rad, 0.5_mps, 0.0_mps, 0.0_rad_per_s, 5.0_s ); },
    [this] (void) -> void { AutoAngle( -60 ); },
    [this] (void) -> void { AimAndPrepShoot( 4.0_s ); },
    [this] (void) -> void { Shoot( 1.0_s ); },
  
    //[this] (void) -> void { Wait( 1.0_s ); },
    [this] (void) -> void { DriveForDistance( -0.7_m, 0.0_m, 0.0_rad, 0.9_mps, 0.0_mps, 0.0_rad_per_s, 5.0_s ); },
    [this] (void) -> void { DriveForDistance( 0.0_m, 0.0_m, -1.15_rad, 0.0_mps, 0.0_mps, 1.0_rad_per_s, 5.0_s ); },
    [this] (void) -> void { MoveArmForPickup(); },
    [this] (void) -> void { DriveForDistance( -1.35_m, 0.0_m, 0.0_rad, 1.0_mps, 0.0_mps, 0.0_rad_per_s, 5.0_s ); },
    //[this] (void) -> void { DriveForDistance( 0.0_m, -0.1_m, 0.0_rad, 0.0_mps, 0.8_mps, 0.0_rad_per_s, 5.0_s ); },
    //[this] (void) -> void { DriveForDistance( 0.0_m,   0.1_m, 0.0_rad, 0.0_mps, 0.8_mps, 0.0_rad_per_s, 5.0_s ); },
    [this] (void) -> void { MoveArmForShooting(); },
    [this] (void) -> void { DriveForDistance( 1.55_m, 0.0_m, 0.0_rad, 1.0_mps, 0.0_mps, 0.0_rad_per_s, 5.0_s ); },
    [this] (void) -> void { DriveForDistance( 0.0_m, 0.0_m, 1.15_rad, 0.0_mps, 0.0_mps, 1.0_rad_per_s, 5.0_s ); },
    [this] (void) -> void { DriveForDistance( 0.61_m, -0.1_m, 0.0_rad, 0.9_mps, 0.5_mps, 0.0_rad_per_s, 5.0_s ); },
    [this] (void) -> void { AimAndPrepShoot( 2.0_s ); },
    [this] (void) -> void { Shoot( 1.0_s ); },
    [this] (void) -> void { DriveForDistance( -0.7_m, 0.0_m, 0.0_rad, 0.9_mps, 0.0_mps, 0.0_rad_per_s, 5.0_s ); },
    [this] (void) -> void { DriveForDistance( 0.0_m, 0.0_m, -1.15_rad, 0.0_mps, 0.0_mps, 1.0_rad_per_s, 5.0_s ); },
    [this] (void) -> void { DriveForDistance( -1.35_m, 0.0_m, 0.0_rad, 1.0_mps, 0.0_mps, 0.0_rad_per_s, 5.0_s ); },
   
  };

  std::vector<std::function<void(void)>> ShootLeftPickupLeftTest = {
    //[this] (void) -> void { DriveForDistance( 0.5_m, 0.0_m, 0.0_rad, 0.5_mps, 0.0_mps, 0.0_rad_per_s, 5.0_s ); },
    [this] (void) -> void { AutoAngle( 60 ); },
    [this] (void) -> void { AimAndPrepShoot( 4.0_s ); },
    [this] (void) -> void { Shoot( 1.0_s ); },
   
    //[this] (void) -> void { Wait( 1.0_s ); },
    [this] (void) -> void { DriveForDistance( 0.7_m, 0.0_m, 0.0_rad, 0.9_mps, 0.0_mps, 0.0_rad_per_s, 5.0_s ); },
    [this] (void) -> void { DriveForDistance( 0.0_m, 0.0_m, 1.15_rad, 0.0_mps, 0.0_mps, 1.0_rad_per_s, 5.0_s ); },
    [this] (void) -> void { MoveArmForPickup(); },
    [this] (void) -> void { DriveForDistance( 1.35_m, 0.0_m, 0.0_rad, 1.0_mps, 0.0_mps, 0.0_rad_per_s, 5.0_s ); },
    //[this] (void) -> void { DriveForDistance( 0.0_m, -0.1_m, 0.0_rad, 0.0_mps, 0.8_mps, 0.0_rad_per_s, 5.0_s ); },
    //[this] (void) -> void { DriveForDistance( 0.0_m,   0.1_m, 0.0_rad, 0.0_mps, 0.8_mps, 0.0_rad_per_s, 5.0_s ); },
    [this] (void) -> void { MoveArmForShooting(); },
    [this] (void) -> void { DriveForDistance( -1.55_m, 0.0_m, 0.0_rad, 1.0_mps, 0.0_mps, 0.0_rad_per_s, 5.0_s ); },
    [this] (void) -> void { DriveForDistance( 0.0_m, 0.0_m, -1.15_rad, 0.0_mps, 0.0_mps, 1.0_rad_per_s, 5.0_s ); },
    [this] (void) -> void { DriveForDistance( -0.61_m, 0.1_m, 0.0_rad, 0.9_mps, 0.5_mps, 0.0_rad_per_s, 5.0_s ); },
    [this] (void) -> void { AimAndPrepShoot( 2.0_s ); },
    [this] (void) -> void { Shoot( 1.0_s ); },
    [this] (void) -> void { DriveForDistance( 0.7_m, 0.0_m, 0.0_rad, 0.9_mps, 0.0_mps, 0.0_rad_per_s, 5.0_s ); },
    [this] (void) -> void { DriveForDistance( 0.0_m, 0.0_m, 1.15_rad, 0.0_mps, 0.0_mps, 1.0_rad_per_s, 5.0_s ); },
    [this] (void) -> void { DriveForDistance( 1.35_m, 0.0_m, 0.0_rad, 1.0_mps, 0.0_mps, 0.0_rad_per_s, 5.0_s ); },
   
  };
    std::vector<std::function<void(void)>> Auto_ShootRunLeft = {
    //[this] (void) -> void { DriveForDistance( 0.5_m, 0.0_m, 0.0_rad, 0.5_mps, 0.0_mps, 0.0_rad_per_s, 5.0_s ); },
    [this] (void) -> void { AutoAngle( 60 ); },
    [this] (void) -> void { AimAndPrepShoot( 4.0_s ); },
    [this] (void) -> void { Shoot( 1.0_s ); },
   
    [this] (void) -> void { DriveForDistance( -0.61_m, 0.0_m, 0.0_rad, 0.9_mps, 0.0_mps, 0.0_rad_per_s, 5.0_s ); },
     [this] (void) -> void { DriveForDistance( 0.0_m, 0.0_m, 1.15_rad, 0.9_mps, 0.0_mps, 0.0_rad_per_s, 5.0_s ); },
    [this] (void) -> void { DriveForDistance( -2.0_m, 0.0_m, 0.0_rad, 0.9_mps, 0.0_mps, 0.0_rad_per_s, 5.0_s ); },
    [this] (void) -> void { Drivetrain_Stop();},
    };

  std::vector<std::function<void(void)>> *autoSequence{ &defaultAutoSequence };

};
