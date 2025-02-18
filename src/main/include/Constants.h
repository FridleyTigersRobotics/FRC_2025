#pragma once

namespace Constants
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

    constexpr int kAlgaeAngleID  = 20;
    constexpr int kCoralAgnleID  = 21;
    constexpr int kElevator0ID   = 22;
    constexpr int kElevator1ID   = 23;
    constexpr int kAlgaeIntakeID = 24;
    constexpr int kCageGrabberID = 25;
    constexpr int kCoralIntakeID = 26;
    constexpr int kClimberEastID    = 27;
    constexpr int kClimberWestID    = 28;
    

    // DIO IDs
    constexpr int kWestEncoderLDIO = 0;
    constexpr int kWestEncoderHDIO = 1;
    constexpr int kEastEncoderLDIO = 2;
    constexpr int kEastEncoderHDIO = 3;
    constexpr int kClimbSwitchDIO = 4;
    constexpr int kCoralEncoderDIO = 5;
    constexpr int kAlgaeEncoderDIO = 6;
    //constexpr int kDIO = 3;
    //constexpr int kDIO = 6;
    //constexpr int kDIO = 7;

    // Encoder Values
    constexpr long kEncClimbUp = 599010;
    constexpr double kClimbCalibrateSpeed = 0.3;
}