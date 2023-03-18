// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;


import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Turntable;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Ramp;
import frc.robot.commands.StopArmElevator;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.Alignment.LimelightPlace;
import frc.robot.commands.ArmMotions.LowGroundGrab;
import frc.robot.commands.ArmMotions.HighGroundGrab;
import frc.robot.commands.ArmMotions.HomePosition;
import frc.robot.commands.ArmMotions.Place;
import frc.robot.commands.ArmMotions.TurntableGrab;
import frc.robot.subsystems.DashboardDisplay;
import frc.robot.subsystems.Swerve;

public class RobotContainer {
    /* Subsystems */
    private final Swerve m_Swerve = new Swerve();
    private final Arm m_Arm = new Arm();
    private final Elevator m_Elevator = new Elevator(0);
    private final Turntable m_Turntable = new Turntable();
    private final Ramp m_Ramp = new Ramp();

    private final Limelight m_Limelight = new Limelight();
    
    private final DashboardDisplay m_Display = new DashboardDisplay(this, m_Swerve, m_Arm, m_Limelight);
    //TODO: Check if I need pneumatics control module

    /* Commands */
    private final StopArmElevator m_StopArmElevator = new StopArmElevator(m_Arm, m_Elevator);

    private final HomePosition m_HomePosition = new HomePosition(m_Arm, m_Elevator);
    private final TurntableGrab m_GrabFromTurntable = new TurntableGrab(m_Arm, m_Elevator, this);
    private final Place m_Place = new Place(this, m_Arm, m_Elevator);
    private final LimelightPlace m_LimelightPlace = new LimelightPlace(m_Swerve, m_Limelight, this, m_Arm, m_Elevator);

    private final LowGroundGrab m_LowGroundGrab = new LowGroundGrab(m_Arm, m_Elevator);
    private final HighGroundGrab m_HighGroundGrab = new HighGroundGrab(m_Arm, m_Elevator);

    /*Control State Variables*/
    private int level = 2;    //Levels 0 - 2 represent FLOOR, MIDDLE, and TOP
    private boolean isCone = true;
    private boolean autoAlign = true;
    private boolean isFrontArm = true;
    private boolean isAlignRight = true;
    
    public int getLevel() { return level; }
    public void setLevel(int level) { this.level = level; }

    public boolean getIsCone() { return isCone; }
    public void setIsCone(boolean isCone) { this.isCone = isCone; }

    public boolean getAutoAlign() { return autoAlign; }
    public void setAutoAlign(boolean autoAlign) { this.autoAlign = autoAlign; }

    public boolean getIsFrontArm() { return isFrontArm; }
    public void setIsArmFront(boolean isArmFront) { this.isFrontArm = isArmFront; }

    public boolean isAlignRight(){ return isAlignRight; }
    public void setIsAlignRight(boolean isAlignRight){ this.isAlignRight = isAlignRight; }

    /*Auton Paths*/
    private FollowPathWithEvents driveForwardPlace;
    private FollowPathWithEvents crossLineDrive;
    private PathPlannerTrajectory traj1;

    /*Robot Container Constructor*/
    public RobotContainer() {
        
        m_Swerve.setDefaultCommand(
                new TeleopSwerve(
                        m_Swerve,
                        () -> -OIConstants.translationSupplier.get(),
                        () -> -OIConstants.strafeSupplier.get(),
                        () -> -OIConstants.rotationSupplier.get(),
                        () -> OIConstants.robotCentric.getAsBoolean(),
                        () -> OIConstants.slowSpeed.getAsBoolean(),
                        () -> OIConstants.centerOfRotation.getAsInt())
                    );
        
        m_Arm.setDefaultCommand(
            new InstantCommand(() -> m_Arm.setArmSpeed(OIConstants.armSpeed.getAsDouble() * 0.3), m_Arm)
        );
            
        m_Elevator.setDefaultCommand(
            new InstantCommand(() -> m_Elevator.setElevatorSpeed(-OIConstants.elevatorSpeed.getAsDouble() * 0.3), m_Elevator)
        );

        m_Turntable.setDefaultCommand(
            new InstantCommand(() -> m_Turntable.setTurntableSpeed(OIConstants.turntableSpeed.getAsDouble() * 0.5), m_Turntable)
        );

        // Configure the button bindings
        configureButtonBindings();
        configureAutoCommands();  
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /*CoDriver Button Bindings*/
        OIConstants.openClaw.onTrue(new InstantCommand(() -> m_Arm.open()));
        OIConstants.closeClaw.onTrue(new InstantCommand(() -> m_Arm.close()));

        OIConstants.increaseLevel.onTrue(new InstantCommand(() -> level = Math.min(level + 1, 2)));
        OIConstants.decreaseLevel.onTrue(new InstantCommand(() -> level = Math.max(level - 1, 0)));

        OIConstants.alignLeft.onTrue(new InstantCommand(() -> isAlignRight = false));
        OIConstants.alignRight.onTrue(new InstantCommand(() -> isAlignRight = true));

        OIConstants.toggleConeCube.onTrue(new InstantCommand(() -> isCone = !isCone));

        OIConstants.toggleRamp.onTrue(new InstantCommand(() -> m_Ramp.toggleRamp()));
        
        OIConstants.turntablePickup.whileTrue(m_GrabFromTurntable.getCommand());
        OIConstants.turntablePickup.onFalse(m_StopArmElevator);
        
        OIConstants.highGround.whileTrue(m_HighGroundGrab.getCommand());
        OIConstants.highGround.onFalse(m_StopArmElevator);

        OIConstants.lowGround.whileTrue(m_LowGroundGrab.getCommand());
        OIConstants.lowGround.onFalse(m_StopArmElevator);

        OIConstants.armHome.whileTrue(m_HomePosition.getCommand());
        OIConstants.armHome.onFalse(m_StopArmElevator);

        /*Main Driver Button Bindings*/
        OIConstants.resetGyro.onTrue(new InstantCommand(() -> m_Swerve.zeroGyro()));
        
        OIConstants.toggleFront.onTrue(
        new SequentialCommandGroup(
            new InstantCommand(() -> isFrontArm = !isFrontArm),
            new InstantCommand(() -> m_Swerve.setIsArmFront(isFrontArm))
        ));

        OIConstants.alignPlace.whileTrue(new ConditionalCommand(m_LimelightPlace.getCommand(), m_Place.getCommand(), () -> autoAlign));
        OIConstants.alignPlace.onFalse(new InstantCommand(() -> m_Limelight.setPipeline(0)));

        OIConstants.calibrateArm.onTrue(new InstantCommand(() -> m_Arm.resetToAbsolute()));
    }

    private void configureAutoCommands()
    {
        /*Pathplanner Setup*/
        HashMap<String, Command> eventMap = new HashMap<>();

        eventMap.put(
            "place", 
            null
            );

        //List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("leftSimple", new PathConstraints(2, 3));
        PathPlannerTrajectory driveForward = PathPlanner.loadPath("driveForward", new PathConstraints(2, 2));
        
        driveForwardPlace = new FollowPathWithEvents(
            m_Swerve.followTrajectoryCommand(driveForward, true),
            driveForward.getMarkers(), 
            eventMap
        );

        PathPlannerTrajectory crossLine = PathPlanner.loadPath("crossLine", new PathConstraints(2, 2));

        crossLineDrive = new FollowPathWithEvents(
            m_Swerve.followTrajectoryCommand(crossLine, true),
            crossLine.getMarkers(), 
            eventMap
        );

        traj1 = PathPlanner.generatePath(
            new PathConstraints(0.5, 0.5), 
            new PathPoint(new Translation2d(0.0, 0), Rotation2d.fromDegrees(0)), // position, heading
            new PathPoint(new Translation2d(1.5, 0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(90)) // position, heading
        );
    }

    public Command getAutonomousCommand() {
        //return m_Swerve.followTrajectoryCommand(traj1, false);
        return null;
    }
}