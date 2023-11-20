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

    // Romi device specific constants
    public static final  double kMaxSpeedMetersPerSec = 0.7; // meters per second
    public static final  double kMaxAngularSpeedRadPerSec = 1.5 * 2 * Math.PI; // 1.5 rotation per second
  
    public static final  double kTrackWidthMeters = 5.551*0.0254; // meters (5.551 inches)
    public static final  double kWheelRadiusMeters = 0.07/2; // meters (d=2.75591 inches, 70 mm)
    public static final  int kEncoderResolution = 1440;

    // For profiled distances PID
    public static final double kPDriveProfiled = 1.2;
    public static final double kIDriveProfiled = 0.0;
    public static final double kDDriveProfiled = 0.0;
    // Max speed and acceleration of the robot
    public static final double kMaxSpeedMetersPerSecond = 0.5;
    public static final double kMaxAccelMetersPerSecondSquared = 1;

    // Debug and tuning enable/disable
    public static final boolean enablePIDTune = true;

    // Positions and increments for PID control goals in meters
    public static final double kStartPosition = 0.0;
    public static final double kminPosition = 0.0;
    public static final double kBackPosition = 0.2; 
    public static final double kFwdPosition = 0.8;
    public static final double kPosIncrement = 0.1;
    public static final double kmaxPosition = 1.0;

    // Feedforward 
    public static final double kSVolts = 0.05; // Static gain
    public static final double kVVoltSecondPerMeter = 1.4; // Velocity gain
    public static final double kAVoltSecondSquaredPerMeter = 0.0; // Acceleration gain
    // public static final double kGVolts = 0; // Gravity gain
    // public static final double kVVoltSecondPerRad = 0.5; // Velocity gain
    // public static final double kAVoltSecondSquaredPerRad = 0.1; // Acceleration gain
       
}
