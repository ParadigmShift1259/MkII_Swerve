// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

//#define ZERO_OFFSETS

#include <numbers>
#include <string>
#include <wpi/DataLog.h>

#include <frc/Encoder.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/AnalogInput.h>
#include <frc/Timer.h>
#include <frc/DataLogManager.h>

#include <rev/CANSparkMax.h>

#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>

#include "Constants.h"

using namespace rev;

class SwerveModule
{
public:
    SwerveModule(int driveMotorCanId, int turningMotorCanId, double offset, bool driveMotorReversed);
    frc::SwerveModuleState GetState();
    frc::SwerveModulePosition GetPosition();
    void SetDesiredState(const frc::SwerveModuleState& state);
    void Periodic();

private:
    units::meters_per_second_t CalcMetersPerSec();
    units::meter_t CalcMeters();

    double VoltageToRadians(double Voltage);

    static constexpr auto kModuleMaxAngularVelocity = std::numbers::pi * 1_rad_per_s;  // radians per second
    static constexpr auto kModuleMaxAngularAcceleration = std::numbers::pi * 2_rad_per_s / 1_s;  // radians per second^2

    static constexpr double kDriveEncoderMetersPerTick = ModuleConstants::kWheelCircumfMeters / ModuleConstants::kEncoderCPR;

    CANSparkMax m_driveMotor;
    CANSparkMax m_turningMotor;

    std::string m_id;

    SparkRelativeEncoder m_turningEncoder = m_turningMotor.GetEncoder(SparkRelativeEncoder::Type::kHallSensor, 42);
    SparkRelativeEncoder m_driveEncoder =  m_driveMotor.GetEncoder(SparkRelativeEncoder::Type::kHallSensor, 42);

    frc::AnalogInput m_absEnc;
    double m_offset = 0.0;
    
    SparkPIDController m_turningPIDController = m_turningMotor.GetPIDController();
    SparkPIDController m_drivePIDController = m_driveMotor.GetPIDController();

    frc::SimpleMotorFeedforward<units::meters> m_driveFeedforward{1_V, 3_V / 1_mps};

    /// Timer used to sync absolute and relative encoders on robot turn on
    frc::Timer m_timer;

    // Logging Member Variables
    wpi::log::DoubleLogEntry m_logTurningEncoderPosition;
    wpi::log::DoubleLogEntry m_logAbsoluteEncoderPosition;
    wpi::log::DoubleLogEntry m_logAbsoluteEncoderPositionWithOffset;
    wpi::log::DoubleLogEntry m_logTurningRefSpeed;
    wpi::log::DoubleLogEntry m_logTurningRefAngle;
    wpi::log::DoubleLogEntry m_logTurningNewSpeed;
    wpi::log::DoubleLogEntry m_logTurningNewAngle;
};
