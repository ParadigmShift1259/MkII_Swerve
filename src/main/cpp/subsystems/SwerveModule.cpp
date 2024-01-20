// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Constants.h"
#include "subsystems/SwerveModule.h"
#include <units/angle.h>

#include <frc/geometry/Rotation2d.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <numbers>

SwerveModule::SwerveModule(const int driveMotorCanId, const int turningMotorCanId, double offset, bool driveMotorReversed)
  : m_driveMotor(driveMotorCanId, rev::CANSparkLowLevel::MotorType::kBrushless)
  , m_turningMotor(turningMotorCanId, rev::CANSparkLowLevel::MotorType::kBrushless)
  , m_id(std::to_string(turningMotorCanId / 2))
  , m_absEnc((turningMotorCanId / 2) - 1)
  , m_offset(offset)
{
  wpi::log::DataLog& log = frc::DataLogManager::GetLog();

  std::string logHeader = "/swerveModule" + m_id + "/";
  m_logTurningEncoderPosition = wpi::log::DoubleLogEntry(log, logHeader + "turningEncoderPosition");
  m_logAbsoluteEncoderPosition = wpi::log::DoubleLogEntry(log, logHeader + "absoluteEncoderPosition");
  m_logAbsoluteEncoderPositionWithOffset = wpi::log::DoubleLogEntry(log, logHeader + "absoluteEncoderPositionWithOffset");
  m_logTurningRefSpeed = wpi::log::DoubleLogEntry(log, logHeader + "refSpeed");
  m_logTurningRefAngle = wpi::log::DoubleLogEntry(log, logHeader + "refAngle");
  m_logTurningNewSpeed = wpi::log::DoubleLogEntry(log, logHeader + "newSpeed");
  m_logTurningNewAngle = wpi::log::DoubleLogEntry(log, logHeader + "newAngle");

  m_turningEncoder.SetPositionConversionFactor(2.0 * std::numbers::pi / DriveConstants::kTurnMotorRevsPerWheelRev); //<! Converts from wheel rotations to radians

  double initPosition = VoltageToRadians(m_absEnc.GetVoltage());
  m_turningEncoder.SetPosition(initPosition);
  printf("Swerve %s initPosition %.3f offset %.3f\n", m_id.c_str(), initPosition, m_offset);
  m_driveMotor.SetSmartCurrentLimit(30);
  m_driveEncoder.SetPosition(0.0);
  m_driveMotor.SetIdleMode(CANSparkMax::IdleMode::kBrake);

  constexpr double kDriveP = 0.0025; // 0.1;
  constexpr double kDriveI = 0;
  constexpr double kDriveD = 0;
  constexpr double kDriveFF = 0.055;//0.047619;
  constexpr double m_max = 1.0;
  constexpr double m_min = -1.0;
  m_drivePIDController.SetP(kDriveP);
  m_drivePIDController.SetI(kDriveI);
  m_drivePIDController.SetD(kDriveD);
  m_drivePIDController.SetFF(kDriveFF);
  m_drivePIDController.SetOutputRange(m_min, m_max);

  m_turningPIDController.SetFeedbackDevice(m_turningEncoder);

  // // Limit the PID Controller's input range between -pi and pi and set the input
  // // to be continuous.
  m_turningPIDController.SetOutputRange(-1.0, 1.0);
  constexpr double kTurnP = 2.0;
  constexpr double kTurnI = 0.0;
  constexpr double kTurnD = 0.0;
  m_turningPIDController.SetP(kTurnP);
  m_turningPIDController.SetI(kTurnI);
  m_turningPIDController.SetD(kTurnD);
  // Do not need m_id for these, use the same value for all swerve pods
  frc::SmartDashboard::PutBoolean("Load Turn PID", false);
  frc::SmartDashboard::PutNumber("Turn P", kTurnP);
  frc::SmartDashboard::PutNumber("Turn I", kTurnI);
  frc::SmartDashboard::PutNumber("Turn D", kTurnD);
  m_turningMotor.SetSmartCurrentLimit(20);
  m_turningMotor.SetIdleMode(CANSparkMax::IdleMode::kBrake);

  frc::SmartDashboard::PutNumber("Set Pos", 0.0);
  
  m_timer.Reset();
  m_timer.Start();
}

void SwerveModule::Periodic()
{
  bool bLoadPID = frc::SmartDashboard::GetBoolean("Load Turn PID", false);
  if (bLoadPID)
  {
    double kTurnP = frc::SmartDashboard::GetNumber("Turn P", 0.5);
    double kTurnI = frc::SmartDashboard::GetNumber("Turn I", 0.00001);
    double kTurnD = frc::SmartDashboard::GetNumber("Turn D", 0.05);
    m_turningPIDController.SetP(kTurnP);
    m_turningPIDController.SetI(kTurnI);
    m_turningPIDController.SetD(kTurnD);
    std::string key = "Swerve " + m_id + " Turn P echo";
    frc::SmartDashboard::PutNumber(key, kTurnP);
    key = "Swerve " + m_id + " Turn I echo";
    frc::SmartDashboard::PutNumber(key, kTurnI);
    key = "Swerve " + m_id + " Turn D echo";
    frc::SmartDashboard::PutNumber(key, kTurnD);
    printf("Swerve %s Loaded Turn PID values P %.3f I %.3f D %.3f\n", m_id.c_str(), kTurnP, kTurnI, kTurnD);
  }

  double absPos = VoltageToRadians(m_absEnc.GetVoltage());
  frc::SmartDashboard::PutNumber("Abs Pos" + m_id, absPos);
  frc::SmartDashboard::PutNumber("Abs Pos Offset" + m_id, m_offset);

  frc::SmartDashboard::PutNumber("Turn Enc Pos" + m_id, m_turningEncoder.GetPosition());
  frc::SmartDashboard::PutNumber("Turn Mot Pos" + m_id, m_turningEncoder.GetPosition() * DriveConstants::kTurnMotorRevsPerWheelRev / (2 * std::numbers::pi));
}

// frc::SwerveModuleState SwerveModule::GetState()
// {
//     double angle = VoltageToRadians(m_absEnc.GetVoltage());
//     return {CalcMetersPerSec(), frc::Rotation2d(units::radian_t(angle))};
// }

frc::SwerveModuleState SwerveModule::GetState()
{
  auto turnPos = m_turningEncoder.GetPosition();
  return {CalcMetersPerSec(), units::radian_t{ turnPos } };
}

units::meters_per_second_t SwerveModule::CalcMetersPerSec()
{
  double velocity = m_driveEncoder.GetVelocity();
  return units::meters_per_second_t(ModuleConstants::kDriveVelocityFactor * velocity);
}


frc::SwerveModulePosition SwerveModule::GetPosition()
{
  auto turnPos = m_turningEncoder.GetPosition();
  return {CalcMeters(), units::radian_t{ turnPos } };
}

units::meter_t SwerveModule::CalcMeters()
{
  double ticks = m_driveEncoder.GetPosition();
  return units::meter_t(kDriveEncoderMetersPerTick * ticks);
}

void SwerveModule::SetDesiredState(const frc::SwerveModuleState& referenceState)
{
  // Optimize the reference state to avoid spinning further than 90 degrees
  double currPosition = m_turningEncoder.GetPosition();
  const auto state = frc::SwerveModuleState::Optimize(referenceState, frc::Rotation2d{ units::radian_t(currPosition) });
  frc::SmartDashboard::PutNumber("Turn Enc Curr Pos" + m_id, currPosition);
  //frc::SmartDashboard::PutNumber("Turn Mot Pos" + m_id, currPosition * kTurnMotorRevsPerWheelRev / (2 * std::numbers::pi));

  if (state.speed != 0_mps)
  {
#ifdef DISABLE_DRIVE
    m_driveMotor.Set(TalonFXControlMode::Velocity, 0.0);
#else
    //double direction = 1.0;//(m_id == "2" || m_id == "3") ? 1.0 : -1.0;
    double direction = (m_id == "1" || m_id == "4") ? -1.0 : 1.0;
    m_driveMotor.Set(state.speed.to<double>() * direction);
    //m_driveMotor.Set(state.speed.to<double>());
#endif
  }
  else
  {
    m_driveMotor.Set(0.0);
  }

  // Calculate the turning motor output from the turning PID controller.
  //frc::SmartDashboard::PutNumber("Turn Ref Opt" + m_id, state.angle.Radians().to<double>());
  //frc::SmartDashboard::PutNumber("Turn Ref" + m_id, referenceState.angle.Radians().to<double>());
//#define USE_TURN_FROM_DASHBOARD
#ifdef USE_TURN_FROM_DASHBOARD
  double newRef = frc::SmartDashboard::GetNumber("Set Pos", 0.0);
#else
  double newRef = state.angle.Radians().to<double>();
#endif

  m_logTurningRefSpeed.Append(referenceState.speed.to<double>());
  m_logTurningRefAngle.Append(referenceState.angle.Degrees().to<double>());
  m_logTurningNewSpeed.Append(state.speed.to<double>());
  m_logTurningNewAngle.Append(state.angle.Degrees().to<double>());
  //m_logTurningNewAngle.Append(newRef);

  //frc::SmartDashboard::PutNumber("Turn Ref Motor" + m_id, newRef);
  m_turningPIDController.SetReference(newRef, CANSparkMax::ControlType::kPosition);
}

double SwerveModule::VoltageToRadians(double Voltage)
{
    frc::SmartDashboard::PutNumber("Voltage" + m_id, Voltage);
    double angle = fmod(Voltage * DriveConstants::kTurnVoltageToRadians - m_offset + 2 * std::numbers::pi, 2 * std::numbers::pi);
    angle = 2 * std::numbers::pi - angle;

    return angle;
}
