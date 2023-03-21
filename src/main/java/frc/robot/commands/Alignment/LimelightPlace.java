// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Alignment;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.commands.DriveTrajectory;
import frc.robot.commands.ArmMotions.Place;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

public class LimelightPlace extends SequentialCommandGroup{
  /** Creates a new Place. */
  private Swerve swerve;
  private Arm arm;
  private Elevator elevator;

  private Limelight limelight;
  private RobotContainer container;

  private PathPlannerTrajectory targetTraj = new PathPlannerTrajectory();
  private DriveTrajectory firstAlign;
  private DriveTrajectory secondAlign;
  private DriveTrajectory driveForward;

  private AlignToAngle alignToAngle1;
  private AlignToAngle alignToAngle2;

  private PreciseAlign preciseAlign;

  public LimelightPlace(Swerve swerve, Limelight limelight, RobotContainer container, Arm arm, Elevator elevator){

    this.swerve = swerve;
    this.arm = arm;
    this.elevator = elevator;

    this.limelight = limelight;
    this.container = container;

    firstAlign = new DriveTrajectory(
        targetTraj,
        swerve::getPose,
        DrivetrainConstants.swerveKinematics,
        new PIDController(7, 0, 0),
        new PIDController(7, 0, 0),
        new PIDController(7, 0, 0),
        swerve::setModuleStates,
        swerve);

    secondAlign = new DriveTrajectory(
        targetTraj,
        swerve::getPose,
        DrivetrainConstants.swerveKinematics,
        new PIDController(7, 0, 0),
        new PIDController(7, 0, 0),
        new PIDController(7, 0, 0),
        swerve::setModuleStates,
        swerve);

    driveForward = new DriveTrajectory(
        targetTraj,
        swerve::getPose,
        DrivetrainConstants.swerveKinematics,
        new PIDController(7, 0, 0),
        new PIDController(7, 0, 0),
        new PIDController(7, 0, 0),
        swerve::setModuleStates,
        swerve);

    preciseAlign = new PreciseAlign(swerve, limelight);

    addCommands(
      new InstantCommand(() -> limelight.setPipeline(0), limelight),
      new WaitCommand(0.3),
      new ConditionalCommand(
        new SequentialCommandGroup(
          new ParallelCommandGroup(
          new Place(container, arm, elevator).getCommand(),
          new SequentialCommandGroup(
            new InstantCommand(() -> targetTraj = GetAlignmentTrajectory(container.getIsCone(), container.isAlignRight(), limelight), limelight),
            new InstantCommand(() -> firstAlign.setTrajectory(targetTraj), swerve),
            firstAlign)
          ),
         // alignToAngle2,
          /*new InstantCommand(() -> targetTraj = GetAlignmentTrajectory(container.getIsCone(), container.isAlignRight(), limelight), limelight),
          new InstantCommand(() -> secondAlign.setTrajectory(targetTraj)),
          secondAlign,*/
          new InstantCommand(() -> targetTraj = GetPlaceTrajectory(container.getIsCone(), container.getLevel(), limelight),limelight),
          new InstantCommand(() -> driveForward.setTrajectory(targetTraj)),
          driveForward,
          new InstantCommand(() -> limelight.setPipeline(container.getIsCone()? 1 : 0), limelight),
          new WaitCommand(1.0),
          preciseAlign
        ),

        new PrintCommand("Target Not Found"),
        ()-> limelight.isTargetFound()
      )
    );
  }

  private PathPlannerTrajectory GetAlignmentTrajectory(boolean isCone, boolean isRightAlign, Limelight limelight) {
    if (isCone) {
      return PathPlanner.generatePath(
          new PathConstraints(4, 1.7),

          new PathPoint(
              swerve.getPose().getTranslation(),
              Rotation2d.fromDegrees(0),
              swerve.getYaw()), // Sets starting point of path to current position.

          new PathPoint(
              swerve.getPose().getTranslation().plus(new Translation2d(limelight.getTz() + 1.2, isRightAlign? -limelight.getTx() - 0.539: -limelight.getTx() + 0.539)),
              Rotation2d.fromDegrees(0),
              swerve.getYaw()));
    }

    else {
      return PathPlanner.generatePath(
          new PathConstraints(4, 1.7),

          new PathPoint(
              swerve.getPose().getTranslation(),
              Rotation2d.fromDegrees(0),
              swerve.getYaw()), // Sets starting point of path to current position.

          new PathPoint(
              swerve.getPose().getTranslation().plus(new Translation2d(limelight.getTz() + 1, -limelight.getTx())),
              Rotation2d.fromDegrees(0),
              swerve.getYaw()));
    }
  }

  private PathPlannerTrajectory GetPlaceTrajectory(boolean isCone, int level, Limelight limelight) {
    switch (level) {
      case 0:
        return PathPlanner.generatePath(
            new PathConstraints(2, 1),

            new PathPoint(
                swerve.getPose().getTranslation(),
                Rotation2d.fromDegrees(0),
                swerve.getYaw()), // Sets starting point of path to current position.

            new PathPoint(
                swerve.getPose().getTranslation().plus(new Translation2d(limelight.getTz() + 0.95, 0)),
                Rotation2d.fromDegrees(0),
                swerve.getYaw()));

      case 1:
        return PathPlanner.generatePath(
            new PathConstraints(2, 1),

            new PathPoint(
                swerve.getPose().getTranslation(),
                Rotation2d.fromDegrees(0),
                swerve.getYaw()), // Sets starting point of path to current position.

            new PathPoint(
                swerve.getPose().getTranslation().plus(new Translation2d(limelight.getTz() + 0.95, 0)),
                Rotation2d.fromDegrees(0),
                swerve.getYaw()));

      case 2:
        return PathPlanner.generatePath(
            new PathConstraints(2, 1),

            new PathPoint(
                swerve.getPose().getTranslation(),
                Rotation2d.fromDegrees(0),
                swerve.getYaw()), // Sets starting point of path to current position.

            new PathPoint(
                swerve.getPose().getTranslation().plus(new Translation2d(limelight.getTz() + 0.52, 0)),
                Rotation2d.fromDegrees(0),
                swerve.getYaw()));

      default:
        return new PathPlannerTrajectory();
    }
  }
}