// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/button/JoystickButton.h>

#include "commands/Autos.h"

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here

  // Configure the button bindings
  ConfigureBindings();
  SetDefaultCommands();
}

void RobotContainer::ConfigureBindings() {
  auto& primary = m_primaryController;

  // Primary
  // Keep the bindings in this order
  // A, B, X, Y, Left Bumper, Right Bumper, Back, Start
  // primary.A().OnTrue(ClawOpen(*this).ToPtr());
  // primary.B().OnTrue(ClawClose(*this).ToPtr());
  // primary.X().OnTrue(RetrieveGamePiece(*this).ToPtr());
  // primary.Y().OnTrue(ReleaseCone(*this).ToPtr());

  primary.LeftBumper().OnTrue(&m_toggleFieldRelative);
  primary.RightBumper().OnTrue(&m_toggleSlowSpeed);

}

// frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
//   // An example command will be run in autonomous
//   return frc2::CommandPtr();
// }

void RobotContainer::SetDefaultCommands()
{
  m_drive.SetDefaultCommand(frc2::RunCommand
  (
    [this] 
    {
        // const double kDeadband = 0.02;
        const double kDeadband = 0.1;
        const auto xInput = frc::ApplyDeadband(m_primaryController.GetLeftY(), kDeadband);
        const auto yInput = frc::ApplyDeadband(m_primaryController.GetLeftX(), kDeadband);
        const auto rotInput = frc::ApplyDeadband(m_primaryController.GetRightX(), kDeadband);      
        const auto rotXInput = frc::ApplyDeadband(m_primaryController.GetRightY(), kDeadband);
        const auto rotYInput = frc::ApplyDeadband(m_primaryController.GetRightX(), kDeadband);

        // Slew rate limiters were running right and down stick actions longer as if coast was on the drive motor???
        const units::meters_per_second_t xSpeed = units::meters_per_second_t{xInput};
        units::meters_per_second_t ySpeed = units::meters_per_second_t{yInput};
        //const auto xSpeed = m_xspeedLimiter.Calculate(xInput) * m_drive.m_currentMaxSpeed; //kMaxSpeed;
        // auto ySpeed = m_yspeedLimiter.Calculate(yInput) * m_drive.m_currentMaxSpeed; //kMaxSpeed;
        auto rot = m_rotLimiter.Calculate(rotInput) * kMaxAngularSpeed;      
        const double rotX = m_rotLimiter.Calculate(rotXInput);
        const double rotY = m_rotLimiter.Calculate(rotYInput);

        frc::SmartDashboard::PutBoolean("FieldRelative", m_fieldRelative);
        if (m_fieldRelative)
        {
          m_drive.RotationDrive(xSpeed, ySpeed, rotX, rotY, m_fieldRelative);
        }
        else
        {
          m_drive.Drive(xSpeed, ySpeed, rot, m_fieldRelative);
        }
    },
    {&m_drive}
  ));
}
