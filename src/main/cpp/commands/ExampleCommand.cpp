// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ExampleCommand.h"

ExampleCommand::ExampleCommand(Drivetrain* subsystem)
    : m_Drivetrain{subsystem} {
  // Register that this command requires the subsystem.
  AddRequirements(m_Drivetrain);
}
