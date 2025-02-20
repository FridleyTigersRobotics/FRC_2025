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



 void Robot::AutonomousInit() {
   
    m_autoSelected = m_autoChooser.GetSelected();

    fmt::print("Auto selected: {}\n", m_autoSelected);

    if (m_autoSelected == kAutoDrive) 
    {
      autoSequence = &Auto_Drive;
    }

    AutonomousStateInit();
    m_autoStateDone = false; 
    m_autoState     = 0;

    Drivetrain_Stop();
   
   
    TeleopInit(); 
    m_fieldRelative = true;
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
    
    m_Drivetrain.Update( GetPeriod() );
    
    m_Climber.Update();
    
   
    
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
  m_initialPose = m_Drivetrain.m_odometry.GetEstimatedPosition();
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
