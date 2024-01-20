// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc2/command/InstantCommand.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/MathUtil.h>

#include "Constants.h"
#include "subsystems/DriveSubsystem.h"

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();

  // frc2::CommandPtr GetAutonomousCommand();

 private:
  void SetDefaultCommands();
  void ConfigureBindings();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  frc2::CommandXboxController m_primaryController{
      OperatorConstants::kDriverControllerPort};

  frc::SlewRateLimiter<units::scalar> m_xspeedLimiter{3 / 1_s, -3 / 2_s};
  frc::SlewRateLimiter<units::scalar> m_yspeedLimiter{3 / 1_s, -3 / 3_s};
  frc::SlewRateLimiter<units::scalar> m_rotLimiter{3 / 1_s};

  // The robot's subsystems are defined here...
  DriveSubsystem m_drive;
  bool m_fieldRelative = false; //true;
  
  frc2::InstantCommand m_toggleFieldRelative{[this] { m_fieldRelative = !m_fieldRelative; }, {}};
  frc2::InstantCommand m_toggleSlowSpeed{[this] { m_drive.ToggleSlowSpeed(); }, {&m_drive}};

};
