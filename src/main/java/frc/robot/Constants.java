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
  public static final class DriveConstants {
    public static final int kLeftMotor1Port = 1;
    public static final int kLeftMotor2Port = 2;
    public static final int kLeftMotor3Port = 3;

    public static final int kRightMotor1Port = 6;
    public static final int kRightMotor2Port = 7;
    public static final int kRightMotor3Port = 8;

    public static final int kEncoderCPR = 1024;
    public static final double kWheelDiameterInches = 6;
    public static final double kEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        (kWheelDiameterInches * Math.PI) / (double) kEncoderCPR;

    public static final boolean kGyroReversed = false;

  
    // Feedforward constants
    public static final double kStaticVoltsStraight = .1; // or is it 10?
    public static final double kVelocityVoltsStraight = 0.1669; // or is it 5.98?

    public static final double kStaticVoltsTurning = 3.14159; // No clue, need to determine
    public static final double kVelocityVoltsTurning = 3.14159; // 


    // Feedback / PID constants
    
    // Different turning and straight PID? Seems reasonable
    public static final double kStraightP = 1;
    public static final double kStraightI = 0;
    public static final double kStraightD = 0;

    public static final double kStraightMaxVelocity = 10; // feet per second
    public static final double kStraightMaxAcceleration = 30; // feet per second squared

    public static final double kStraightTolerancePosition = .1; // feet
    public static final double kStraightToleranceVelocity = .1; // feet per second


    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;    

    public static final double kMaxTurnRateDegPerS = 100;
    public static final double kMaxTurnAccelerationDegPerSSquared = 300;

    public static final double kToleranceDeg = 5;
    public static final double kRateToleranceDegPerS = 10; // degrees per second


    public static final double kNaiveP = 1;
    public static final double kNaiveI = 0;
    public static final double kNaiveD = 0;   

  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
  }
}


