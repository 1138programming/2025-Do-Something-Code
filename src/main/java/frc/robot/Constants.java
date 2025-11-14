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
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class TurretConstants {
    public static final int kTurretMotorPortBase = 1; // CAN ID for Kraken motor (base rotation)
    public static final int kTurretMotorPortAngle = 2; // CAN ID for Neo motor (angle adjustment)
    public static final int kTurretEncoderPortBase = 0;
    public static final int kTurretEncoderPortAngle = 1;

    public static final double kTurretMaxAngle = 85; // degrees - range = 0-85
    public static final double kTurretMinAngle  = 0; // degrees - range = 0-3
    public static final double kTurretBaseMaxAngle = 300; 
    public static final double kTurretBaseMinAngle = 0;

    public static final double kTrapezoidTimeIntervale = 0.02; // seconds

    public static final double kTurretBaseP = 0.1; // PID proportional gain
    public static final double kTurretBaseI = 0.0; // PID integral gain
    public static final double kTurretBaseD = 0.0; // PID derivative gain

    public static final double kTurretAngleP = 0.1; // PID proportional gain
    public static final double kTurretAngleI = 0.0; // PID integral gain
    public static final double kTurretAngleD = 0.0; // PID derivative gain

    public static final double kBaseMaxVoltage = 12.0; // Maximum voltage for base motor
    public static final double kBaseMaxAcceleration = 60; // Minimum voltage for base motor
    public static final double kAngleMaxVoltage = 12.0; // Maximum voltage for angle motor

    
    public static final double kSpeedBaseModifier = 0.6; // Modifier for base rotation speed
    public static final double kSpeedAngleModifier = 0.6; // Modifier for angle adjustment speed

    //encoder
    public static final double kTurretAngleEncoderFullRotationValue = 360; 
    public static final double kTurretAngleEncoderZeroPosition = 0; //use as future offset values when drifting
  }
}
