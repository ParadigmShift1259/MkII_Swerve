// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <numbers>

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */
namespace DriveConstants{

    constexpr double kTurnVoltageToRadians = 2.0 * std::numbers::pi / 4.93;    // Absolute encoder runs 0 to 4.93V
    constexpr double KTurnVoltageToDegrees = 360 / 4.93;

    constexpr double kDriveGearRatio = 8.31;                //!< MK2 swerve modules 11.9 ft/sec
    //constexpr double kDriveGearRatio = 8.16;                //!< MK3 swerve modules w/NEOs 12.1 ft/sec
    //constexpr double kDriveGearRatio = 6.86;                //!< MK3 swerve modules w/NEOs 14.4 ft/sec
    constexpr double kTurnMotorRevsPerWheelRev = 18.0;

}

namespace ModuleConstants
{
    constexpr double kWheelDiameterMeters = .1016;    // 4"
    constexpr double kEncoderCPR = 42.0;
    
    constexpr double kWheelCircumfMeters = (kWheelDiameterMeters * std::numbers::pi);
    // Assumes the encoders are directly mounted on the wheel shafts
    constexpr double kDriveVelocityFactor = kWheelCircumfMeters / 60.0;

    // Assumes the encoders are directly mounted on the wheel shafts
    //constexpr double kTurningEncoderDistancePerPulse = (std::numbers::pi * 2) / static_cast<double>(kEncoderCPR);

    constexpr double kP_ModuleTurningController = 1.1;
    constexpr double kD_ModuleTurningController = 0.03;

    constexpr double kPModuleDriveController = 0.001;

    constexpr int kMotorCurrentLimit = 30;
}   // namespace ModuleConstants


namespace OperatorConstants {

constexpr int kDriverControllerPort = 0;

}  // namespace OperatorConstants
