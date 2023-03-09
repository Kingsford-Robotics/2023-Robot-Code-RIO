//Based on Team 364 Swerve Drive Code and modified to extend functionality.
    
package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import frc.robot.SwerveModule;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.RobotConstants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import edu.wpi.first.wpilibj2.command.*;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    public SwerveDrivePoseEstimator swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public PigeonIMU gyro;

    public boolean isArmFront = true;

    private Translation2d centerOfRotation = new Translation2d(0, 0);

    private ShuffleboardTab SwerveTab;
    public GenericEntry robotX;
    public GenericEntry robotY;

    public Swerve() {
        gyro = new PigeonIMU(RobotConstants.pigeonID);
        gyro.configFactoryDefault();
        zeroGyro();

        SwerveTab = Shuffleboard.getTab("Swerve");
        robotX = SwerveTab.add("Robot X", 0).getEntry();
        robotY = SwerveTab.add("Robot Y", 0).getEntry();

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, DrivetrainConstants.Mod0.constants),
            new SwerveModule(1, DrivetrainConstants.Mod1.constants),
            new SwerveModule(2, DrivetrainConstants.Mod2.constants),
            new SwerveModule(3, DrivetrainConstants.Mod3.constants)
        };

        /* By pausing init for a second before setting module offsets, we avoid a bug with inverting motors.
         * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
         */
        Timer.delay(1.0);
        resetModulesToAbsolute();

        swerveOdometry = new SwerveDrivePoseEstimator(DrivetrainConstants.swerveKinematics, getYaw(), getModulePositions(), new Pose2d(0, 0, new Rotation2d(0)));
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates;
        
        if(fieldRelative)
        {
            swerveModuleStates = DrivetrainConstants.swerveKinematics.toSwerveModuleStates(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getYaw()
                                ), centerOfRotation);
        }

        //TODO: Check if isArmFront is correct
        else{
            swerveModuleStates = DrivetrainConstants.swerveKinematics.toSwerveModuleStates(
                new ChassisSpeeds(
                                    isArmFront? -translation.getX(): translation.getX(),
                                    isArmFront? -translation.getY(): translation.getY(), 
                                    rotation
                                ), centerOfRotation);
        }
        
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DrivetrainConstants.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DrivetrainConstants.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }    

    public Pose2d getPose() {
        return swerveOdometry.getEstimatedPosition();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro(){
        gyro.setYaw(0);
    }

    public void setIsArmFront(boolean isArmFront){
        this.isArmFront = isArmFront;
    }

    public Rotation2d getYaw() {
       return (DrivetrainConstants.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getYaw()) : Rotation2d.fromDegrees(gyro.getYaw());
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    public static enum RotationCenter
    {
        CENTER, FRONTLEFT, FRONTRIGHT, BACKLEFT, BACKRIGHT
    }

    public void setCenterOfRotation(RotationCenter center)
    {
        switch(center)
        {
            case CENTER:
                centerOfRotation = new Translation2d(0, 0);
                break;
            case FRONTLEFT:
                centerOfRotation = DrivetrainConstants.frontLeftLocation;
                break;
            case FRONTRIGHT:
                centerOfRotation = DrivetrainConstants.frontRightLocation;
                break;
            case BACKLEFT:
                centerOfRotation = DrivetrainConstants.backLeftLocation;
                break;
            case BACKRIGHT:
                centerOfRotation = DrivetrainConstants.backRightLocation;
                break;

        }
    }


public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
   return new SequentialCommandGroup(
        new InstantCommand(() -> {
          // Reset odometry for the first path you run during auto
          if(isFirstPath){
              this.resetOdometry(traj.getInitialHolonomicPose());
          }
        }),
        new PPSwerveControllerCommand(
            traj, 
            this::getPose, // Pose supplier
            DrivetrainConstants.swerveKinematics, // SwerveDriveKinematics
            new PIDController(1, 0.1, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            new PIDController(1, 0.1, 0), // Y controller (usually the same values as X controller)
            new PIDController(2, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            this::setModuleStates, // Module states consumer
            true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
            this // Requires this drive subsystem
        )
    );
}

    @Override
    public void periodic(){
        swerveOdometry.update(getYaw(), getModulePositions()); 
        robotX.setDouble(swerveOdometry.getEstimatedPosition().getX());
        robotY.setDouble(swerveOdometry.getEstimatedPosition().getY()); 
    }
}