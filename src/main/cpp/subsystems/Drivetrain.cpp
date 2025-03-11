// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#define UseElasticNetTable 1

#include "subsystems/Drivetrain.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <fmt/printf.h>
#include <math.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>

//pathplanner includes
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/config/RobotConfig.h>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/DriverStation.h>
#include <frc/TimedRobot.h>

using namespace pathplanner;

frc::ChassisSpeeds CurrentChassisSpeeds = frc::ChassisSpeeds{ units::meters_per_second_t {0.0}, units::meters_per_second_t {0.0}, units::radians_per_second_t {0.0} };

Drivetrain::Drivetrain() 
{
    ResetIMU();
    ConfigureAutoBuilder();

  #if UseElasticNetTable
  DriveNetTable = DriveNetInst.GetTable("2227 Drivetrain");
  DriveNetInst.StartServer();
  #endif
    
}

void Drivetrain::AutonomousInit()
{
  ResetIMU(); //reset imu here in case robot is moved after powerup
}


void Drivetrain::drive(frc::ChassisSpeeds chassisSpeedinput, bool fieldRelative,
                       units::second_t period) {
  frc::ChassisSpeeds CurrentChassisSpeeds = chassisSpeedinput;
  units::meters_per_second_t xSpeed = CurrentChassisSpeeds.vx;
  units::meters_per_second_t ySpeed = CurrentChassisSpeeds.vy;
  units::radians_per_second_t rot = CurrentChassisSpeeds.omega;
  auto states =
      m_kinematics.ToSwerveModuleStates(frc::ChassisSpeeds::Discretize(
          fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                              xSpeed, ySpeed, rot, m_imu.GetRotation2d())
                        : frc::ChassisSpeeds{xSpeed, ySpeed, rot},
          period));

  m_kinematics.DesaturateWheelSpeeds(&states, kMaxSpeed);

  auto [fl, fr, bl, br] = states;

  m_frontLeft.SetDesiredState(fl);
  m_frontRight.SetDesiredState(fr);
  m_backLeft.SetDesiredState(bl);
  m_backRight.SetDesiredState(br); 

  CurrentChassisSpeeds = frc::ChassisSpeeds{xSpeed, ySpeed, rot};
}

void Drivetrain::UpdateOdometry() {
  m_odometry.Update(m_imu.GetRotation2d(),
                    {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
                     m_backLeft.GetPosition(), m_backRight.GetPosition()});
}

frc::Pose2d Drivetrain::getPose()
{
  return m_odometry.GetPose();
}

void Drivetrain::resetPose(frc::Pose2d poseinput)
{
  m_odometry.ResetPose(poseinput);
}

frc::ChassisSpeeds Drivetrain::getRobotRelativeSpeeds()
{
  return CurrentChassisSpeeds;
}

void Drivetrain::Periodic()
{
  UpdateOdometry();
}

void Drivetrain::UpdateSmartDashboardData()
{
  #if UseElasticNetTable
  // The velocity is the Euclidean norm of the linear velocities
  float xvel = m_imu.GetVelocityX();
  float yvel = m_imu.GetVelocityY();
  float rvel = std::sqrt( xvel * xvel + yvel * yvel );
  DriveNetTable->PutNumber("Robot meters per second", rvel );
  #endif
}

void Drivetrain::TeleopInit()
{

}

void Drivetrain::ResetIMU()
{
  m_imu.Reset();
}


//PathPlanner
void Drivetrain::ConfigureAutoBuilder()
{
    RobotConfig config = RobotConfig::fromGUISettings();
    AutoBuilder::configure(
        [this](){ return getPose(); }, // Robot pose supplier
        [this](frc::Pose2d pose){ resetPose(pose); }, // Method to reset odometry (will be called if your auto has a starting pose)
        [this](){ return getRobotRelativeSpeeds(); }, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        [this](auto speeds, auto feedforwards){ drive(speeds, false, frc::TimedRobot::kDefaultPeriod); }, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
        std::make_shared<PPHolonomicDriveController>( // PPHolonomicController is the built in path following controller for holonomic drive trains
            PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
            PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
        ),
        config, // The robot configuration
        []() {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            auto alliance = frc::DriverStation::GetAlliance();
            if (alliance) {
                return alliance.value() == frc::DriverStation::Alliance::kRed;
            }
            return false;
        },
        this
        ); // Reference to this subsystem to set requirements
}













/*

frc::ChassisSpeeds CurrentChassisSpeeds = frc::ChassisSpeeds{ units::meters_per_second_t {0.0}, units::meters_per_second_t {0.0}, units::radians_per_second_t {0.0} };


Drivetrain::Drivetrain() 
{
    m_imu.ResetDisplacement(); 
    m_imu.Reset();
}

void Drivetrain::SetFieldRelative( bool fieldRelative )
{
  m_fieldRelative = fieldRelative;
}



void Drivetrain::Periodic() 
{
  units::second_t period{ frc::TimedRobot::kDefaultPeriod };
  frc::ChassisSpeeds ChassisSpeedsToUse;
  m_frontLeft.UpdateEncoders();
  m_frontRight.UpdateEncoders();
  m_backLeft.UpdateEncoders();
  m_backRight.UpdateEncoders();

  if ( m_fieldRelative )
  {
    ChassisSpeedsToUse = \
      frc::ChassisSpeeds::FromFieldRelativeSpeeds(
        m_xSpeed, 
        m_ySpeed, 
        m_rot,
        frc::Rotation2d{ units::degree_t { -GetYaw() }}
      );
  }
  else
  {
    ChassisSpeedsToUse = frc::ChassisSpeeds{ m_xSpeed, m_ySpeed, m_rot };
  }
        frc::SmartDashboard::PutNumber("THE ROT aka input rotation command to chassis", float{m_rot});
  frc::ChassisSpeeds ChassisSpeeds = frc::ChassisSpeeds::Discretize( ChassisSpeedsToUse, period );

  auto states = m_kinematics.ToSwerveModuleStates( ChassisSpeeds );

  CurrentChassisSpeeds = ChassisSpeeds;

  m_kinematics.DesaturateWheelSpeeds( &states, kMaxSpeed );

  auto [fl, fr, bl, br] = states;

  
  //double minimumSpeed = 0.01;

  // Check if the wheels don't have a drive velocity to maintain the current wheel orientation.
  bool hasVelocity = true;
    fabs( double{fl.speed} ) <= minimumSpeed || 
    fabs( double{fr.speed} ) <= minimumSpeed || 
    fabs( double{bl.speed} ) <= minimumSpeed || 
    fabs( double{br.speed} ) <= minimumSpeed;

  frc::SmartDashboard::PutNumber( "fl.speed", double{fl.speed} );
  frc::SmartDashboard::PutNumber( "fr.speed", double{fr.speed} );
  frc::SmartDashboard::PutNumber( "bl.speed", double{bl.speed} );
  frc::SmartDashboard::PutNumber( "br.speed", double{br.speed} );

  if ( !hasVelocity )
  {
    fl.angle = m_frontLeft.GetState().angle;
    fr.angle = m_frontRight.GetState().angle;
    bl.angle = m_backLeft.GetState().angle;
    br.angle = m_backRight.GetState().angle;
  }

  m_frontLeft.SetDesiredState(fl);
  m_frontRight.SetDesiredState(fr);
  m_backLeft.SetDesiredState(bl);
  m_backRight.SetDesiredState(br);
  UpdateSmartDashboardData();
}

void Drivetrain::drive(units::meters_per_second_t xSpeed,
                       units::meters_per_second_t ySpeed,
                       units::radians_per_second_t rot, bool fieldRelative,
                       units::second_t period) 
                       {
  //auto states = can add this back in if need to use states within this function, removed to get rid of compiler warning of it being set but not used
      m_kinematics.ToSwerveModuleStates(frc::ChassisSpeeds::Discretize(
          fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                              xSpeed, ySpeed, rot,
                              m_odometry.GetEstimatedPosition().Rotation())
                        : frc::ChassisSpeeds{xSpeed, ySpeed, rot},
          period));
          
          }



void Drivetrain::SetSpeeds(
  units::meters_per_second_t  xSpeed,
  units::meters_per_second_t  ySpeed, 
  units::radians_per_second_t rot
)
{
  m_xSpeed = xSpeed;
  m_ySpeed = ySpeed;
  m_rot    = rot;
}

frc::ChassisSpeeds Drivetrain::getRobotRelativeSpeeds()  //This can be calculated using one of WPILib's drive kinematics classes or read from IMU
{
bool useKinematics = true;

if(!useKinematics)
{
  units::meters_per_second_t vx = units::meters_per_second_t{1.0_mps} * m_imu.GetVelocityX();
  units::meters_per_second_t vy = units::meters_per_second_t{1.0_mps} * m_imu.GetVelocityY();
  units::radians_per_second_t omega = units::radians_per_second_t {0.017453293_rad_per_s} * m_imu.GetRate();// 1 deg/sec = pi/180 rad/sec
  return (frc::ChassisSpeeds {vx, vy, omega});
}
else
{
  return CurrentChassisSpeeds;
}


}

frc::Pose2d Drivetrain::getPose()
{
  return m_odometry.GetEstimatedPosition();
}

void Drivetrain::resetPose(frc::Pose2d poseinput)
{
  m_odometry.ResetPose(poseinput);
}

void Drivetrain::UpdateSmartDashboardData()
{
  #if 1
  frc::SmartDashboard::PutNumber( "Drive_X", double{m_odometry.GetEstimatedPosition().X()});
  frc::SmartDashboard::PutNumber( "Drive_y", double{m_odometry.GetEstimatedPosition().Y()});
  frc::SmartDashboard::PutNumber( "Drive_Rot", double{m_odometry.GetEstimatedPosition().Rotation().Degrees()});
  frc::SmartDashboard::PutNumber( "Yawby bobby", m_imu.GetYaw());
  #endif
}

void Drivetrain::TeleopInit()
{

}


void Drivetrain::UpdateOdometry() {
  m_odometry.Update( frc::Rotation2d{ units::degree_t{ GetYaw() } },
                    { m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
                      m_backLeft.GetPosition(), m_backRight.GetPosition()});
}

void Drivetrain::Init(){
}

void Drivetrain::ResetYaw()
{
  m_imu.ZeroYaw();
  
  feature allowing the driver to "reset"
  the "yaw" angle.  When the reset occurs, the new gyro angle will be
  0 degrees.  This can be useful in cases when the gyro drifts, which
  doesn't typically happen during a FRC match, but can occur during
  long practice sessions.
  
}



double Drivetrain::GetYaw(){
  return m_imu.GetYaw();
}


double Drivetrain::GetAngle(){
  return m_imu.GetAngle();
}


*/

