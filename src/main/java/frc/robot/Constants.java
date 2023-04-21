// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.config.SwerveModuleConstants;
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    
    public static final class ModuleConstants {
    public static final double kMaxModuleAngularSpeedRadiansPerSecond = 2 * Math.PI;
    public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 2 * Math.PI;

    public static final int kEncoderCPR = 1024;
    public static final double kWheelDiameterMeters = 0.1016;
    public static final double kDriveEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;

    public static final double kTurningEncoderDistancePerPulse =
        // Assumes the encoders are on a 1:1 reduction with the module shaft.
        (2 * Math.PI) / (double) kEncoderCPR;

    public static final double kPModuleTurningController = 1;

    public static final double kPModuleDriveController = 1;
  }



 
  public static final class Swerve {
    public static final double fastDriveSpeedMultiplier = 1.0;
    public static final double normalDriveSpeedMultiplier = 0.6;
    public static final double slowDriveSpeedMultiplier = 0.05;

    //SET ME Each Run...
    public static final double robotOffset = 0.0;
    
    public static final double stickDeadband = 0.1;

    public static final boolean invertGyro = true; // Always ensure Gyro is CCW+ CW-

    /* Drivetrain Constants */
    public static final double trackWidth = Units.inchesToMeters(17.5);
    public static final double wheelBase = Units.inchesToMeters(19);
    public static final double wheelDiameter = Units.inchesToMeters(4.0);
    public static final double wheelCircumference = wheelDiameter * Math.PI;

    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    public static final double driveGearRatio =
        (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0); // 6.75:1
    public static final double angleGearRatio = (48.0 / 40.0); //
    public static final SwerveDriveKinematics swerveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0));

    /* Swerve Voltage Compensation */
    public static final double voltageComp = 12.0;

    /* Swerve Current Limiting */
    public static final int angleContinuousCurrentLimit = 20;
    public static final int driveContinuousCurrentLimit = 80;

    /* Angle Motor PID Values */
    public static final double angleKP = 0.01;
    public static final double angleKI = 0.0;
    public static final double angleKD = 0.0;
    public static final double angleKFF = 0.0;

    /* Drive Motor PID Values */
    public static final double driveKP = 0.1;
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKFF = 0.0;

    /* Drive Motor Characterization Values */
    public static final double driveKS = 0.667;
    public static final double driveKV = 2.44;
    public static final double driveKA = 0.27;

    /* Drive Motor Conversion Factors */
    public static final double driveConversionPositionFactor =
        (wheelDiameter * Math.PI) / driveGearRatio;
    public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
    public static final double angleConversionFactor = 360.0 / angleGearRatio;

    /* Swerve Profiling Values */
    public static final double maxSpeed = 4.5; // meters per second  //4.5
    public static final double maxAngularVelocity = 6; // 11.5

    /* Neutral Modes */
    public static final IdleMode angleNeutralMode = IdleMode.kBrake;
    public static final IdleMode driveNeutralMode = IdleMode.kBrake;

    /* Motor Inverts */
    public static final boolean driveInvert = true;
    public static final boolean angleInvert = true;

    /* Angle Encoder Invert */
    public static final boolean canCoderInvert = false;

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 {
      public static final int driveMotorID = 12;
      public static final int angleMotorID = 13;
      //public static final int canCoderID = 4;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(42.58);
      //public static final Rotation2d angleOffset = Rotation2d.fromDegrees(43.8);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, angleOffset);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 {
      public static final int driveMotorID = 22;
      public static final int angleMotorID = 23;
      //public static final int canCoderID = 1;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(296.67);
      //public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-29.69);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, angleOffset);
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 {
      public static final int driveMotorID = 15;
      public static final int angleMotorID = 14;
      //public static final int canCoderID = 3;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(62.99);
      //public static final Rotation2d angleOffset = Rotation2d.fromDegrees(204.7);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, angleOffset);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 {
      public static final int driveMotorID = 25;
      public static final int angleMotorID = 24;
      //public static final int canCoderID = 2;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(470.5);
     //public static final Rotation2d angleOffset = Rotation2d.fromDegrees(9.2);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, angleOffset);
    }
  }


}
  


