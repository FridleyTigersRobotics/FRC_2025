#include <Robot.h>
#include <frc/MathUtil.h>
#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <fmt/printf.h>
#include <frc/filter/SlewRateLimiter.h>

#include <Drivetrain.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <string>
#include <Climber.h>
#include <networktables/NetworkTable.h>
#include <LimelightHelpers.h>
#include <cameraserver/CameraServer.h>


#if NEW_AUTO_CHOOSER
std::vector<std::string> AutoModeNames = {
    "DO NOTHING",
    "Drive",
    "ShootCenter",
    "ShootCenterPickupCenter",
    "ShootLeftPickupLeft",
    "ShootRightPickupRight",
    "CenterShootRun",
    "ShootRightPickupRightTest",
    "ShootLeftPickupLeftTest",
    "ShootRunLeft"
};

std::string Robot::TranslateAutoModeToAutoString( uint32_t autoModeInt ) {
    std::string autoString = kAutoNameDefault;
    
    if ( autoModeInt < AutoModeNames.size() )
    {
      autoString = AutoModeNames[autoModeInt];
    }

    return autoString;
}

#endif

 void Robot::AutonomousInit() {
   
  #if NEW_AUTO_CHOOSER
    m_autoSelectedInteger = frc::SmartDashboard::GetNumber("AutoModeInt", 0 );
    fmt::print("Auto selected integer: {}\n", m_autoSelectedInteger);
    m_autoSelected = TranslateAutoModeToAutoString( m_autoSelectedInteger );
  #else
    m_autoSelected = m_autoChooser.GetSelected();
  #endif

    fmt::print("Auto selected: {}\n", m_autoSelected);

    if (m_autoSelected == kAutoDrive) 
    {
      autoSequence = &Auto_Drive;
    }
    else if (m_autoSelected == kShootCenter) 
    {
      autoSequence = &Auto_ShootCenter;
    }
    else if (m_autoSelected == kShootCenterPickupCenter) 
    {
      autoSequence = &Auto_ShootCenterPickupCenter;
    }
     else if (m_autoSelected == kShootLeftPickupLeft) 
    {
      autoSequence = &ShootLeftPickupLeft;
    }
     else if (m_autoSelected == kShootRightPickupRight) 
    {
      autoSequence = &ShootRightPickupRight;
    }
     else if (m_autoSelected == kCenterShootRun) 
    {
      autoSequence = &Auto_CenterShootRun;
    }
    else if (m_autoSelected == kShootRightPickupRightTest) 
    {
      autoSequence = &ShootRightPickupRightTest;
    }
    else if (m_autoSelected == kShootLeftPickupLeftTest) 
    {
      autoSequence = &ShootLeftPickupLeftTest;
    }
     else if (m_autoSelected == kShootRunLeft) 
    {
      autoSequence = &Auto_ShootRunLeft;
    }



    AutonomousStateInit();
    m_autoStateDone = false; 
    m_autoState     = 0;

    Drivetrain_Stop();
   
   
   
    m_Climber.ChangeClimberState( m_Climber.ClimberStop );
    

    TeleopInit(); 
    m_fieldRelative = false;
    m_autoTimer.Stop();
    m_autoTimer.Reset();
    m_autoTimer.Start();
    m_AutoXdirPid.SetTolerance( kXyPosTolerance,  kXyVelTolerance );
    m_AutoYdirPid.SetTolerance( kXyPosTolerance,  kXyVelTolerance );
    m_AutoRotatePid.SetTolerance(  kRotPosTolerance, kRotVelTolerance );

    m_AutoXdirPid.Reset( 0.0_m );
    m_AutoYdirPid.Reset( 0.0_m );
    m_AutoRotatePid.Reset( 0.0_rad );

 }

  void Robot::AutonomousPeriodic() {
    AutonomousStateUpdate();
    RunAutoSequence();
    
    m_Drivetrain.updateDrivetrain( GetPeriod(), m_fieldRelative );
    
    m_Climber.updateClimber();
    
   
    
  }





void Robot::AutoAngle(double AngleCrap) {
  m_Drivetrain.m_YawOffset    = -AngleCrap;
  m_DriveTargetAngle          = -AngleCrap;
  m_autoStateDone = true;


  //Goal(facing away)=the current angle (if right (-1*120) jf left (+120))
}


void Robot::Drivetrain_Stop() {
   m_Drivetrain.SetSpeeds( 0.0_mps, 0.0_mps, 0.0_rad_per_s );
}






 void Robot::DriveForDistance( 
  units::meter_t              xDistance, 
  units::meter_t              yDistance, 
  units::radian_t             rotRadians, 
  units::meters_per_second_t  xSpeedMult, 
  units::meters_per_second_t  ySpeedMult, 
  units::radians_per_second_t rotSpeedMult,
  units::time::second_t       maxTime
)
{
  
}


void Robot::MoveArmForPickup()
{
 
 
  m_autoStateDone = true;
}

void Robot::MoveArmForShooting()
{
 

  if ( m_autoTimer.Get() > 1.0_s )
  {
    
    m_autoStateDone = true;
  }
}



void Robot::AimAndPrepShoot( units::second_t maxTime )
{
  
  //Drive for distance using the limlight aiming. We can use the same thing as we do in tele-op?
  //Yoink the variables from teleop in robot? hmm...
  
  


  {
    m_autoStateDone = true;
  }
#if 0

  double angleToTurnTo = tx / 180.0 * (std::numbers::pi*2);

  DriveForDistance( 0.0_m, 0.0_m, angleToTurnTo );
  m_autoStateDone = true;
#endif
}

void Robot::Shoot( units::second_t maxTime )
{
 

  if ( m_autoTimer.Get() > maxTime )
  {
   
    m_autoStateDone = true;
  }
}


void Robot::Wait( units::second_t maxTime )
{
  if ( m_autoTimer.Get() > maxTime )
  {
    m_autoStateDone = true;
  }
}

void Robot::AutonomousStateInit()
{
  m_AutoXdirPid.Reset(0.0_m);
  m_AutoYdirPid.Reset(0.0_m);
  m_AutoRotatePid.Reset(0.0_rad);

  m_autoTimer.Stop();
  m_autoTimer.Reset();
  m_autoTimer.Start();
  
  m_Drivetrain.m_odometry.ResetPosition(
    frc::Rotation2d{units::degree_t {m_Drivetrain.GetYaw()}},
    {m_Drivetrain.m_frontLeft.GetPosition(), m_Drivetrain.m_frontRight.GetPosition(),
     m_Drivetrain.m_backLeft.GetPosition(),  m_Drivetrain.m_backRight.GetPosition()},
    frc::Pose2d{}
  );
  m_initialPose = m_Drivetrain.m_odometry.GetPose();
}


void Robot::AutonomousStateUpdate()
{

}



void Robot::RunAutoSequence()
{
  if ( m_autoStateDone )
  {
    m_autoState++;
    m_autoStateDone = false;
    AutonomousStateInit();
  }

  // frc::SmartDashboard::PutNumber("Auto_Idx",  m_autoState);
  if ( m_autoState < (*autoSequence).size() )
  {
    (*autoSequence)[m_autoState]();
  }
}

// i can smell you