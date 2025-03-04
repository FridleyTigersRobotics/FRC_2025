// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

namespace Constants
{
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
    constexpr int kCoralAngleID  = 21;
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

    // Climber Control Values
    constexpr long kEncClimbUp = 636791;
    constexpr double kClimbCalibrateSpeed = 0.3;
    constexpr double kClimbSpeed = 0.75;
    constexpr double kGrabSpeed = 0.75;
    constexpr double kGrabberPidP{16.0e-2};
    constexpr double kGrabberPidI{8.0e-1};
    constexpr double kGrabberPidD{0.00};
    constexpr double kGrab90{11.00};

    // Coral Intake Control Values
    constexpr double kCoralAnglePidP     = 3.00;
    constexpr double kCoralAnglePidI     = 0.50;
    constexpr double kCoralAnglePidD     = 0.00;
    constexpr double kCoralIntakeSpeed   = 1.0;
    constexpr double kCoralAngleUp       = 0.410;
    constexpr double kCoralAngleDn       = 0.710;
    constexpr double kCoralAngleVertical = 0.652;
    constexpr double kCoralAngleSpeed    = 0.75;
    constexpr double kCoralAnglePlace    = 0.507;
    constexpr double kCoralAngleTopPlace = 0.445;
    constexpr double kCoralAngleHorizontal = 0.410;


    // Algae Intake Control Values
    constexpr double kAlgaeAnglePidP     = 3.00;
    constexpr double kAlgaeAnglePidI     = 0.50;
    constexpr double kAlgaeAnglePidD     = 0.00;
    constexpr double kAlgaeIntakeSpeed   = 0.5;
    constexpr double kAlgaeAngleUp       = 0.750;
    constexpr double kAlgaeAngleDn       = 0.50;
    constexpr double kAlgaeAngleSpeed    = 0.75;


    //Elevator Control Values
    constexpr double kPosStart = 0.00;
    constexpr double kPosCoralL1 = 9.15;//L1 level = 46cm from carpet
    constexpr double kPosCoralL2 = 30.57;//L2 level = 81cm from carpet 
    constexpr double kPosCoralL3 = 62.70;//L3 level = 121cm from carpet
    constexpr double kPosCoralL4 = 98.5;//L4 level = 183cm from carpet, but different angle
    constexpr double kPosCoralIntake = 34.14;//station = 95cm bottom of opening first set to value 37, testing showed needed to drop 3cm for value 34.14

    constexpr double kElevatorLowestAlgae = 25.0;

    constexpr double kElevatorPidP = {0.10};
    constexpr double kElevatorPidI = {0.00};
    constexpr double kElevatorPidD ={0.00};
    constexpr double kElevatorSpeed = 0.5;

    //other control values
    constexpr bool kJoystickFieldRelative = true; //operate joystick controls as field relative if true

}


namespace OperatorConstants {

inline constexpr int kDriverControllerPort = 0;

}  // namespace OperatorConstants
