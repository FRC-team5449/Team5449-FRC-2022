// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  
  // CAN IDs
  public static final int LeftFMotorID = 1;
  public static final int RightFMotorID = 2;
  public static final int LeftBMotorID = 3;
  public static final int RightBMotorID = 4;
  public static final int LeftFServoID = 21;
  public static final int RightFServoID = 22;
  public static final int LeftBServoID = 23;
  public static final int RightBServoID = 24;
  public static final int LeftFEncoderID = 31;
  public static final int RightFEncoderID = 32;
  public static final int LeftBEncoderID = 33;
  public static final int RightBEncoderID = 34;
  public static final int Intake1 = 5;
  public static final int Intake2 = 6;
  public static final int Intake3 = 7;
  public static final int Intake4 = 8;
  public static final int turretAngle = 9;
  public static final int turretDirection = 10;
  public static final int Shooter = 11;
  public static final int PneumaticHub = 15;

  // DifferentialModelType
  public static enum DifferentialModelType{
      LEFTMODEL, RIGHTMODEL;
  }

  // Swerve Chassis
  public static final double ChassisWheelDistancePerPulse = 0.000019095266283;
  public static final double driveKs = 0.0;
  public static final double driveKv = 0.28;
  public static final double turnKs = 0.28317;
  public static final double turnKv = 1.1362E-05;
  public static final double kMaxSpeed = 6.0; // meters per second


  // Drive Mode
  public static enum DriveMode{
      AP, TANKDRIVE, ARCADEDRIVE, SWERVEDRIVE, CLIMBUP
  }

  // Autonomous mode
  public static enum AutoMode{
      AP, PATH_BOTTOM, PATH_BOTTOM_HOMING, PATH_BOTTOM_MIDDLE_HOMING, PATH_BOTTOM_MIDDLE, PATH_MIDDLE, PATH_MIDDLE_HOMING, PATH_MIDDLE_BOTTOM_HOMING, PATH_MIDDLE_BOTTOM, PATH_TOP, PATH_TOP_MIDDLE, PATH_MIDDLE_TOP, PATH_BOTTOM_TOP_HOMING
  }

  // Ball Niche
  public static enum BallNiche{
    RED, BLUE, EMPTY
  }

  // Shooter
  public static final double ShooterMaxSpeed = 20000;  // Shooter Max Speed in pulse/100ms
  public static final double EmissionMaxSpeed = 9.5;   // Ball Emission Max Speed in meters/second
  public static final double ShooterSpeedError = 500;  // Shooter Speed Error in pulse/100ms

  // Turret
  public static final double MaxTurretAngle = 65;
  public static final double MinTurretAngle = 30;
  public static final double TurretAnglePulseRange = 680;
}
