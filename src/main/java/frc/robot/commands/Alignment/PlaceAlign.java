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
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

public class PlaceAlign extends CommandBase {
  /** Creates a new AlignCone. */

  Swerve swerve;
  Limelight limelight;
  RobotContainer container;

  PathPlannerTrajectory targetPath;
  
  public PlaceAlign(RobotContainer container, Swerve swerve, Limelight limelight) {
    this.swerve = swerve;
    this.limelight = limelight;
    this.container = container;

    addRequirements(swerve, limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    targetPath = PathPlanner.generatePath(
    new PathConstraints(1, 1),
    
    new PathPoint(
      swerve.getPose().getTranslation(), 
      Rotation2d.fromDegrees(0), 
      swerve.getYaw()
    ), //Sets starting point of path to current position.

    new PathPoint(
      swerve.getPose().getTranslation().minus(new Translation2d(limelight.getTz() - 1, -limelight.getTx())), 
      Rotation2d.fromDegrees(0), 
      Rotation2d.fromDegrees(0.0)) // position, heading(direction of travel), holonomic rotation
    );

    swerve.followTrajectoryCommand(targetPath, false).schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
      
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.drive(
      new Translation2d(0.0, 0.0), 
      0.0, 
      false, 
      true
    );
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
