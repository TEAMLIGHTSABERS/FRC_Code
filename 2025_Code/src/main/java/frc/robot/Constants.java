// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {


  /*2024 Constants
  public static final class Turret {
    public static final int kTurCanId = 5;
    public static final int kTurCurrentLimit = 47;
    public static final double kIEController = 0.0; //0.000001; //DO NOT GO > 0.000001
    public static final double kDEController = 0.0;

    public static final double kPEGainStartUp = 0.0; 
    public static final double kVEGainStartUp = 0.003;
    public static final double kPEGainStartDown = 0.0; 
    public static final double kVEGainStartDown = 0.002;
    //public static final double kPEGainSpeakerUp = 0.0; 
    //public static final double kVEGainSpeakerUp = 0.005;
    //public static final double kPEGainSpeakerDown = 0.0; 
    //public static final double kVEGainSpeakerDown = 0.003;
    public static final double kPEGainHighShotUp = 0.0; 
    public static final double kVEGainHighShotUp = 0.006;
    public static final double kPEGainHighShotDown = 0.0; 
    public static final double kVEGainHighShotDown = 0.003;
    public static final double kPEGainAmpUp = 0.0; 
    public static final double kVEGainAmpUp = 0.003;


    public static final int kStartID = 0;
    //public static final int kSpeakerID = 1;
    public static final int kHighShotID = 2;
    public static final int kAmpID = 3;

    public static final double kStartPosition = 15; // Degrees
    //public static final double kLongShotPosition = 45; // Degrees
    public static final double kHighShotPosition = 55; // Degrees  Actual speaker position
    public static final double kAmpPosition = 125; // Degrees

    public static int kDeflID = 0;
    public static double kDeflON = 0.0;
    public static double kDeflOff = 1.0;
  }

  public static final class Intake {
    public static double kTopPower = 1.0;
    public static double kFeedPower = 0.5;
    public static double kShotFeedTime = 1.0;
  }

  public static final class Launcher {
    public static double kInputDelaySec = 5;
    public static double kLeftPower = 1.0;
    public static double kRightPower = 1.0;
    public static double kTimeToLaunch = 1.0;
    public static double kTimeToStop = 3.0; // seconds when Launching
    public static double kFlyWheelStopTime = 1.0; // Seconds when Spinning Up
    public static double kFeederSpeed = 0.5;
    public static double kWheelRateRatio = 4.0/3.0;

    //Wheel Rates
    
    public static double kAmpL = 500;
    public static double kAmpR = 500;

    public static double kHighShotL = 3000;
    public static double kHighShotR = 3000;

    public static double kSpeakerL = 5400;
    public static double kSpeakerR = 5000;

    public static double kStartL = 5400;
    public static double kStartR = 5000;

  
    // Gen Pid K
    public static final int kCurrentLimit = 80;

    public static double kpP = 0.00007;
    public static double kpI = 0.0000007;
    public static double kpD = 0;
    public static double kpIz = 0;
    public static double kpFF = 0.000007;
    
    public static final int kLTSCanId = 3;
    public static final int kRTSCanId = 4;

    public static final int kPLTSCanId = 8;
    public static final int kPRTSCanId = 9;
  }*/
  
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 3.0; //(default 4.8)
    public static final double kMaxAngularSpeed = Math.PI; // radians per second (default 2 * Math.PI )

    public static final double kDirectionSlewRate = 1.2; // radians per second
    public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(23.75);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(21.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 13;
    public static final int kRearLeftDrivingCanId = 15;
    public static final int kFrontRightDrivingCanId = 11;
    public static final int kRearRightDrivingCanId = 17;

    public static final int kFrontLeftTurningCanId = 14;
    public static final int kRearLeftTurningCanId = 16;
    public static final int kFrontRightTurningCanId = 12;
    public static final int kRearRightTurningCanId = 18;


    public static final boolean kGyroReversed = false;
    
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 13;

    /*// Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;*/

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    /*public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps*/
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.05;
    public static final double kTriggerButtonThreshold = 0.25;
    public static final double kInputDelayTimedOut = 25; // Cycle Counts -> 0.5 sec
    public static final double kDriveAngleDeadband = 5; // degrees
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }
}
