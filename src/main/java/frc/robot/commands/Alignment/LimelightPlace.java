// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Alignment;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.ArmMotions.Place;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

public class LimelightPlace extends SequentialCommandGroup {
  /** Creates a new Place. */

  Swerve swerve;
  Limelight limelight;
  RobotContainer container;
  Arm arm;
  Elevator elevator;

  PathPlannerTrajectory targetTraj;
  
  public LimelightPlace(Swerve swerve, Limelight limelight, RobotContainer container, Arm arm, Elevator elevator) {

    this.swerve = swerve;
    this.limelight = limelight;
    this.container = container;

    this.arm = arm;
    this.elevator = elevator;

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AlignToAngle(swerve, 0),

      new InstantCommand(() -> targetTraj = PathPlanner.generatePath(
        new PathConstraints(1, 1),
        
        new PathPoint(
          swerve.getPose().getTranslation(), 
          Rotation2d.fromDegrees(0), 
          swerve.getYaw()
        ), //Sets starting point of path to current position.
    
        new PathPoint(
          swerve.getPose().getTranslation().plus(new Translation2d(limelight.getTz() + 1, -limelight.getTx())),
          Rotation2d.fromDegrees(0),
          swerve.getYaw()
        )
        )
    ),

    swerve.followTrajectoryCommand(targetTraj, false),

    new InstantCommand(() -> targetTraj = PathPlanner.generatePath(
        new PathConstraints(1, 1),
        
        new PathPoint(
          swerve.getPose().getTranslation(), 
          Rotation2d.fromDegrees(0), 
          swerve.getYaw()
        ), //Sets starting point of path to current position.
    
        new PathPoint(
          swerve.getPose().getTranslation().plus(new Translation2d(limelight.getTz() + 1, -limelight.getTx())),
          Rotation2d.fromDegrees(0),
          swerve.getYaw()
        )
        )
      ),

      //This might dynamically create new command at runtime and run it I hope.
      new InstantCommand(() -> swerve.followTrajectoryCommand(targetTraj, false).asProxy()),

      new Place(container, arm, elevator).getCommand()
      //Add command to drive forward to place and then release. Add code to align with tx offset when close. Add code for cones offset.


    );
  }
}