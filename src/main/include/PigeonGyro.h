#pragma once

//#define WPI_IGNORE_DEPRECATED
//#include <wpi/deprecated.h>
//#define WPI_UNIGNORE_DEPRECATED

#include <units/angular_velocity.h>
//#include <ctre/phoenix.h>

#define USE_PIGEON_2
#ifdef USE_PIGEON_2
#include <ctre/phoenix6/Pigeon2.hpp>
#else
#include <ctre/phoenix/sensors/PigeonIMU.h>
#endif

class PigeonGyro
{
public:
    PigeonGyro();

    frc::Rotation2d GetRotation2d();
    double GetPitch() { return m_gyro.GetPitch().GetValue().to<double>(); }
    void Reset();
    void Set(units::degree_t yaw);
    units::degrees_per_second_t GetTurnRate();
         
#ifdef USE_PIGEON_2
    ctre::phoenix6::hardware::Pigeon2 m_gyro;
#else
    PigeonIMU m_gyro;
#endif
};

