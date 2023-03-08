// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//Based on Team 364 Code and Modified

package frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

/** Add your docs here. */
public final class DrivetrainConstants {
    public static final boolean invertGyro = false;

    public static final COTSFalconSwerveConstants chosenModule = COTSFalconSwerveConstants
            .SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L2);

    /* Drivetrain Constants */
    public static final double trackWidth = Units.inchesToMeters(20.75);
    public static final double wheelBase = Units.inchesToMeters(27.25);
    public static final double wheelCircumference = chosenModule.wheelCircumference;

    public static final Translation2d frontLeftLocation = new Translation2d(wheelBase / 2.0, trackWidth / 2.0);
    public static final Translation2d frontRightLocation = new Translation2d(wheelBase / 2.0, -trackWidth / 2.0);
    public static final Translation2d backLeftLocation = new Translation2d(-wheelBase / 2.0, trackWidth / 2.0);
    public static final Translation2d backRightLocation = new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0);

    /* Swerve Kinematics */
    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            frontLeftLocation,
            frontRightLocation,
            backLeftLocation,
            backRightLocation);

    /* Module Gear Ratios */
    public static final double driveGearRatio = chosenModule.driveGearRatio;
    public static final double angleGearRatio = chosenModule.angleGearRatio;

    /* Motor Inverts */
    public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
    public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;

    /* Angle Encoder Invert */
    public static final boolean canCoderInvert = chosenModule.canCoderInvert;

    /* Swerve Current Limiting */
    public static final int angleContinuousCurrentLimit = 20;
    public static final int anglePeakCurrentLimit = 30;
    public static final double anglePeakCurrentDuration = 0.1;
    public static final boolean angleEnableCurrentLimit = true;

    public static final int driveContinuousCurrentLimit = 35;
    public static final int drivePeakCurrentLimit = 60;
    public static final double drivePeakCurrentDuration = 0.1;
    public static final boolean driveEnableCurrentLimit = true;

    /*
     * These values are used by the drive falcon to ramp in open loop and closed
     * loop driving.
     * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc
     */
    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    /* Angle Motor PID Values */
    public static final double angleKP = chosenModule.angleKP;
    public static final double angleKI = chosenModule.angleKI;
    public static final double angleKD = chosenModule.angleKD;
    public static final double angleKF = chosenModule.angleKF;

    /* Drive Motor PID Values */
    public static final double driveKP = 0.05; // TODO: This must be tuned to specific robot
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKF = 0.0;

    /*
     * Drive Motor Characterization Values
     * Divide SYSID values by 12 to convert from volts to percent output for CTRE
     */
    public static final double driveKS = (0.32 / 12); // TODO: This must be tuned to specific robot
    public static final double driveKV = (1.51 / 12);
    public static final double driveKA = (0.27 / 12);

    /* Swerve Profiling Values */
    /** Meters per Second */
    public static final double maxSpeed = 5; // TODO: This must be tuned to specific robot
    /** Radians per Second */
    public static final double maxAngularVelocity = 10.0; // TODO: This must be tuned to specific robot

    /* Neutral Modes */
    public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
    public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 {
        public static final int driveMotorID = 1;
        public static final int angleMotorID = 2;
        public static final int canCoderID = 1;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(189.22);
        public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                canCoderID, angleOffset);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 {
        public static final int driveMotorID = 3;
        public static final int angleMotorID = 4;
        public static final int canCoderID = 2;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(107.3);
        public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                canCoderID, angleOffset);
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 {
        public static final int driveMotorID = 5;
        public static final int angleMotorID = 6;
        public static final int canCoderID = 3;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(167.4);
        public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                canCoderID, angleOffset);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 {
        public static final int driveMotorID = 7;
        public static final int angleMotorID = 8;
        public static final int canCoderID = 4;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(256.46);
        public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                canCoderID, angleOffset);
    }
}