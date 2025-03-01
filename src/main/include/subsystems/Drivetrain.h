// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <numbers>

#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <studica/AHRS.h>
#include "subsystems/SwerveModule.h"
#include "Constants.h"
#include <frc/SPI.h>
#include <frc2/command/sysid/SysIdRoutine.h>

using namespace Constants;

class Drivetrain : public frc2::SubsystemBase {
 public:
  Drivetrain();
  
   void drive(frc::ChassisSpeeds chassisSpeedinput,
             bool fieldRelative, units::second_t period);
  void UpdateOdometry();
  void Periodic( ) override;

  static constexpr units::meters_per_second_t kMaxSpeed =
      3.0_mps;  // 3 meters per second
  static constexpr units::radians_per_second_t kMaxAngularSpeed{
      std::numbers::pi};  // 1/2 rotation per second

  frc::ChassisSpeeds getRobotRelativeSpeeds();

  frc::Pose2d getPose();

  void resetPose(frc::Pose2d poseinput);
 
  void UpdateSmartDashboardData();

  void TeleopInit();

  void ResetIMU();

 private:
  frc::Translation2d m_frontLeftLocation {+0.28575_m, +0.28575_m};
  frc::Translation2d m_frontRightLocation{+0.28575_m, -0.28575_m};
  frc::Translation2d m_backLeftLocation  {-0.28575_m, +0.28575_m};
  frc::Translation2d m_backRightLocation {-0.28575_m, -0.28575_m};

  SwerveModule m_backRight { kBackRightDriveID,  kBackRightSpinID,  kDriveEncoderBackRight,  (std::numbers::pi + -4.97), kMaxSpeed, "BR" };
  SwerveModule m_frontRight{ kFrontRightDriveID, kFrontRightSpinID, kDriveEncoderFrontRight, (std::numbers::pi + -2.75), kMaxSpeed, "FR" };
  SwerveModule m_backLeft  { kBackLeftDriveID,   kBackLeftSpinID,   kDriveEncoderBackLeft,   (std::numbers::pi + -3.35), kMaxSpeed, "BL" };
  SwerveModule m_frontLeft { kFrontLeftDriveID,  kFrontLeftSpinID,  kDriveEncoderFrontLeft,   (std::numbers::pi + -5.6), kMaxSpeed, "FL" };

  studica::AHRS m_imu { studica::AHRS::NavXComType::kMXP_SPI };

  frc::SwerveDriveKinematics<4> m_kinematics{
      m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation,
      m_backRightLocation};

  frc::SwerveDriveOdometry<4> m_odometry{
      m_kinematics,
      m_imu.GetRotation2d(),
      {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
       m_backLeft.GetPosition(), m_backRight.GetPosition()}};

  void ConfigureAutoBuilder();



































/*
  void SetFieldRelative( bool fieldRelative );


  //void Periodic( units::second_t period, bool fieldRelative ) override;
  void Periodic( ) override;
  void drive(units::meters_per_second_t xSpeed,
             units::meters_per_second_t ySpeed, units::radians_per_second_t rot,
             bool fieldRelative, units::second_t period);
  void SetSpeeds(
    units::meters_per_second_t  xSpeed,
    units::meters_per_second_t  ySpeed, 
    units::radians_per_second_t rot
    );

  void TeleopInit();
  void UpdateOdometry();
  void Init();
  void ResetYaw();
  void UpdateSmartDashboardData();
  double GetYaw();
  double GetAngle();
  frc::ChassisSpeeds getRobotRelativeSpeeds();
  frc::Pose2d getPose();
  void resetPose(frc::Pose2d poseinput);


  static constexpr units::meters_per_second_t kMaxSpeed =
      0.2_mps;  // 3 meters per second


  static constexpr units::radians_per_second_t kMaxAngularSpeed{
      0.5 * std::numbers::pi};  // 1/2 rotation per second

 
   units::meters_per_second_t  m_xSpeed{ 0.0 };
   units::meters_per_second_t  m_ySpeed{ 0.0 };
   units::radians_per_second_t m_rot   { 0.0 };


  //------------|back|-------------
  /// 32 in diagonal
  // 22.627417
  //11.3137085
  frc::Translation2d m_frontLeftLocation {+0.28575_m, +0.28575_m};
  frc::Translation2d m_frontRightLocation{+0.28575_m, -0.28575_m};
  frc::Translation2d m_backLeftLocation  {-0.28575_m, +0.28575_m};
  frc::Translation2d m_backRightLocation {-0.28575_m, -0.28575_m};

//1.230863 Drive motor #10     1.230863 / (2*std::numbers::pi),      
//0.909437 Drive motor #12     0.5 + 0.909437 / (2*std::numbers::pi),
//0.255626 Drive motor #14     0.255626 / (2*std::numbers::pi),      
//4.980153 Drive motor #16     4.980153 / (2*std::numbers::pi),      

  SwerveModule m_backRight { kBackRightDriveID,  kBackRightSpinID,  kDriveEncoderBackRight,  -4.97, kMaxSpeed, "BR" };
  SwerveModule m_frontRight{ kFrontRightDriveID, kFrontRightSpinID, kDriveEncoderFrontRight, -2.75, kMaxSpeed, "FR" };
  SwerveModule m_backLeft  { kBackLeftDriveID,   kBackLeftSpinID,   kDriveEncoderBackLeft,   -3.35, kMaxSpeed, "BL" };
  SwerveModule m_frontLeft { kFrontLeftDriveID,  kFrontLeftSpinID,  kDriveEncoderFrontLeft,   -5.6, kMaxSpeed, "FL" };
                                                                //0.79    1.58
  studica::AHRS m_imu { studica::AHRS::NavXComType::kMXP_SPI };

 public:
   frc::SwerveDriveKinematics<4> m_kinematics{
      m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation,
      m_backRightLocation};




    // X Postive = Backwards
    // Y Positive = Right
    // Rot Positive = Clockwise

  frc::SwerveDrivePoseEstimator<4> m_odometry{
      m_kinematics,
      frc::Rotation2d{units::degree_t {GetYaw()}},//m_gyro.GetRotation2d(),
      {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
       m_backLeft.GetPosition(), m_backRight.GetPosition()},
      frc::Pose2d{},
      {0.1, 0.1, 0.1},
      {0.1, 0.1, 0.1}};
*/

};
