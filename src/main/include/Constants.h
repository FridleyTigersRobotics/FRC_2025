#pragma once

namespace ConstantCrap
{
   //-----------|Front|------------
  // 2                           3
  //  7-6--------------------8-9
  //  |----YAY CODE TEAM!------|
  //  |------------------------|
  //  |------------------------|
  //  |------Tom is smart------|
  //  |------Eli is smart------|
  //  |----Mina is smartish----|
  //  |----Kewsar is smart-----|
  //  |----Hail Bing Skrong----|
  //  |-Bing Skrong is strong--|
  //  5-4--------------------3-2
//  1                           0

    // Analog IDs
    constexpr int kDriveEncoderBackRight  = 0;
    constexpr int kDriveEncoderBackLeft   = 1;
    constexpr int kDriveEncoderFrontLeft  = 2;
    constexpr int kDriveEncoderFrontRight = 3;

    // Can IDs
    constexpr int kFrontLeftSpinID      = 6;
    constexpr int kFrontLeftDriveID     = 7;
    constexpr int kBackLeftSpinID       = 4;
    constexpr int kBackLeftDriveID      = 5;
    constexpr int kFrontRightSpinID     = 9;
    constexpr int kFrontRightDriveID    = 8;
    constexpr int kBackRightSpinID      = 2;
    constexpr int kBackRightDriveID     = 3;

    // TODO : Check all these, wrist an motor both 18?
    constexpr int kIntakeMotorcanID     = 22;
    constexpr int kArmMotorLeftcanID    = 21;
    constexpr int kArmMotorRightcanID   = 19;
    constexpr int kWristMotorID         = 20;
    constexpr int kShooterMotorID       = 18;
    constexpr int kRightClimberMotor    = 4;
    constexpr int kLeftClimberMotor     = 3;

    // DIO IDs
    constexpr int kArmEncoderDIO           = 0;
    constexpr int kWristEncoderDIO         = 1;
    constexpr int kLeftClimberEncoderDIO1  = 4;
    constexpr int kLeftClimberEncoderDIO2  = 5;
    constexpr int kRightClimberEncoderDIO1 = 2;
    constexpr int kRightClimberEncoderDIO2 = 3;
    constexpr int kRightClimberStopDIO     = 6;
    constexpr int kLeftClimberStopDIO      = 7;


}