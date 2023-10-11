// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

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
    public static final int kManipulatorControllerPort = 1; 
  }

  public static class DrivetrainConstants {
    // Wheels
    public static class FrontLeft {
      public static final int kRotate = 9; 
      public static final int kDrive = 2; 
      public static final int kRotEncoder = 10; 
    }
    public static class FrontRight {
      public static final int kRotate = 3; 
      public static final int kDrive = 4; 
      public static final int kRotEncoder = 11; 
    }
    public static class BackRight {
      public static final int kRotate = 5; 
      public static final int kDrive = 6; 
      public static final int kRotEncoder = 12; 
    }
    public static class BackLeft {
      public static final int kRotate = 8; 
      public static final int kDrive = 7; 
      public static final int kRotEncoder = 13; 
    }

    // Gearing & Conversions
    public static final double kGearRatio = 6.8; 
    public static final double kWheelRadiusInches = 1.5; 
    public static final double kMetersPerRot = Units.inchesToMeters(2 * Math.PI * kWheelRadiusInches / kGearRatio);
    public static final double kMetersPerSecondPerRPM = kMetersPerRot / 60;

    public static final double kRotateGearRatio = 1; 
    public static final double kDegreesPerRot = 360 / kRotateGearRatio;
    public static final double kDegreesPerSecondPerRPM = kDegreesPerRot / 60; 

    // Drivebase
    public static final double kTrackWidthMeters = Units.inchesToMeters(17.5); 
    public static final double kWheelBaseMeters = Units.inchesToMeters(17.5); 

    // Kinematics
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
      new Translation2d(kTrackWidthMeters / 2, kWheelBaseMeters / 2), // front left
      new Translation2d(kTrackWidthMeters / 2, -kWheelBaseMeters / 2), // front right
      new Translation2d(-kTrackWidthMeters / 2, kWheelBaseMeters / 2), // back left
      new Translation2d(-kTrackWidthMeters / 2, -kWheelBaseMeters / 2) // back right
    ); 

    // Speeds (v2) (https://github.com/SwerveDriveSpecialties/swerve-template-2021-unmaintained/blob/master/src/main/java/frc/robot/subsystems/DrivetrainSubsystem.java)
    public static final double kMaxAttainableSpeedMetersPerSecond = 5880.0 / 60.0 / kGearRatio *
    2 * Units.inchesToMeters(kWheelRadiusInches) * Math.PI; 
    public static final double kMaxAttainableRotationRadPerSecond = kMaxAttainableSpeedMetersPerSecond /
    Math.hypot(kTrackWidthMeters / 2.0, kWheelBaseMeters / 2.0); // max rotation of robot
    
    // TODO: tune these
    public static final double kMaxSpeedMetersPerSecond = 1.0; // max velocity (no turning) of robot; may tune to be a fraction of the attainable module speed
    public static final double kMaxAccelerationMetersPerSecondSquared = kMaxSpeedMetersPerSecond / 1.0; // max acceleration of robot (accelerate to max speed in 1 second)
    public static final double kMaxRotationRadPerSecond = Math.PI; 
    public static final double kMaxRotationAccelerationRadPerSecondSquared = Math.PI; // max angular acceleration of robot

    // NO NEED to tune these
    public static final double ksVolts = 0; 
    public static final double kvVoltSecondsPerMeter = 0; 
    public static final double kaVoltSecondsSquaredPerMeter = 0; 

    // // per-swerve-module turn feedforward values; calculate by running SysID on one of the turn swerve modules
    // // make sure to run on the carpet
    // public static final double ksTurnVolts = 0; 
    // public static final double kvTurnVoltSecondsPerMeter = 0; 
    // public static final double kaTurnVoltSecondsSquaredPerMeter = 0; 


    // drivetrain sysid (lock wheels)
    public static final double kModuleDrive_P = 0; 
    public static final double kModuleDrive_I = 0; 
    public static final double kModuleDrive_D = 0; 

    // found from sysid for one of the turn modules or tune by yourself
    public static final double kModuleTurn_P = 0.0039; 
    public static final double kModuleTurn_I = 0; 
    public static final double kModuleTurn_D = 0.0017; 

    // turn in place pid
    public static final double kTurn_P = 0; 
    public static final double kTurn_I = 0; 
    public static final double kTurn_D = 0; 
    public static final double kTurn_FF = 0; 
    public static final double kTurnErrorThreshold = 0; 
    public static final double kTurnVelocityThreshold = 0; 

    // TODO: tune these but it should be fine
    public static final int kDriveCurrentLimit = 40; 
    public static final int kTurnCurrentLimit = 20; 

    public static final double kForwardSlewRate = kMaxAccelerationMetersPerSecondSquared; 
    public static final double kStrafeSlewRate = kMaxAccelerationMetersPerSecondSquared; 
    public static final double kTurnSlewRate = kMaxRotationAccelerationRadPerSecondSquared; 
  }

  public static class Trajectory {
    public static final double kDrive_P = 0; 
    public static final double kDrive_I = 0; 
    public static final double kDrive_D = 0;

    // drivetrain angular sysid
    public static final double kOmega_P = DrivetrainConstants.kTurn_P; 
    public static final double kOmega_I = DrivetrainConstants.kTurn_I; 
    public static final double kOmega_D = DrivetrainConstants.kTurn_D; 

    public static final double kMaxVelocityMetersPerSecond = 3.21; 
    public static final double kMaxAccelerationMetersPerSecondSquared = 2.54; 

    public static final double kMaxCentripetalAcceleration = 0.8; 

    public static final HashMap<String, Command> eventMap = new HashMap<>(); 

    static {
      // add events to map
    }
  }
  public static class ArmConstants{
    public static double kFF = 0;
    public static final double kS = 0;
    public static final double kV = 0;
    public static final double kG = 0;
    public static final double kA = 0;
    public static final double kT = 0;
    public static final double kArmLength = 0;
    public static final double kArmWeight = 0;
    public static final double kMortorOhms = 0;
    public static final double kGearBox = 0;
    public static final double kInitialAngle = 120;
    public static final double kGroundAngle = 0;
    public static final double kP = 0;
    public static final int kCurrentLimit = 40;
    public static final double kConversionFactor = 0.5;
    public static final double kVelocityConversionFactor = kConversionFactor/60;
    public static final double kAngleError = 0;
  }

}
