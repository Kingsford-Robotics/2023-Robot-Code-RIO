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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.commands.DriveTrajectory;
import frc.robot.commands.ArmMotions.Place;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

public class LimelightPlace {
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

  public LimelightPlace(Swerve swerve, Limelight limelight, RobotContainer container, Arm arm, Elevator elevator) {

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

    alignToAngle1 = new AlignToAngle(swerve, 0);
    alignToAngle2 = new AlignToAngle(swerve, 0);

    preciseAlign = new PreciseAlign(swerve, limelight);

  }

  public SequentialCommandGroup getCommand() {
    List<CommandBase> commandList = new ArrayList<CommandBase>();
    SequentialCommandGroup group;

    // Sets pipeline AprilTag for initial alignment.
    commandList.add(
        new InstantCommand(() -> limelight.setPipeline(0)));

    commandList.add(
        alignToAngle1
    );

    if (limelight.isTargetFound()) {
      commandList.add(
          new InstantCommand(() -> targetTraj = GetAlignmentTrajectory(container.getIsCone(), limelight), limelight)
      );

      commandList.add(
          new InstantCommand(() -> firstAlign.setTrajectory(targetTraj), swerve)
      );

      commandList.add(
          firstAlign
      );

      commandList.add(
        alignToAngle2
      );

      commandList.add(
        new InstantCommand(() -> targetTraj = GetAlignmentTrajectory(true, limelight),
        limelight)
      );

      commandList.add(
        new InstantCommand(() -> secondAlign.setTrajectory(targetTraj))
      );

      commandList.add(
        secondAlign
      );

      commandList.add(
          new Place(container, arm, elevator).getCommand()
      );

      commandList.add(
        new InstantCommand(
          () -> targetTraj = GetPlaceTrajectory(container.getIsCone(), container.getLevel(), limelight),
          limelight
        )
      );

      commandList.add(
          new InstantCommand(() -> driveForward.setTrajectory(targetTraj))
      );

      commandList.add(
          driveForward
      );

      if (container.getIsCone()) {
        commandList.add(
            new InstantCommand(() -> limelight.setPipeline(1))
        );
      }

      commandList.add(
          preciseAlign
      );

      commandList.add(
        new InstantCommand(() -> limelight.setPipeline(0))
      );
    }

    group = new SequentialCommandGroup(commandList.toArray(new CommandBase[commandList.size()]));
    return group;
  }

  private PathPlannerTrajectory GetAlignmentTrajectory(boolean isCone, Limelight limelight) {
    if (isCone) {
      return PathPlanner.generatePath(
          new PathConstraints(1, 1),

          new PathPoint(
              swerve.getPose().getTranslation(),
              Rotation2d.fromDegrees(0),
              swerve.getYaw()), // Sets starting point of path to current position.

          new PathPoint(
              swerve.getPose().getTranslation().plus(new Translation2d(limelight.getTz() + 1, -limelight.getTx())),
              Rotation2d.fromDegrees(0),
              swerve.getYaw()));
    }

    else {
      return PathPlanner.generatePath(
          new PathConstraints(1, 1),

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
            new PathConstraints(1, 1),

            new PathPoint(
                swerve.getPose().getTranslation(),
                Rotation2d.fromDegrees(0),
                swerve.getYaw()), // Sets starting point of path to current position.

            new PathPoint(
                swerve.getPose().getTranslation().plus(new Translation2d(limelight.getTz() + 1, -limelight.getTx())),
                Rotation2d.fromDegrees(0),
                swerve.getYaw()));

      case 1:
        return PathPlanner.generatePath(
            new PathConstraints(1, 1),

            new PathPoint(
                swerve.getPose().getTranslation(),
                Rotation2d.fromDegrees(0),
                swerve.getYaw()), // Sets starting point of path to current position.

            new PathPoint(
                swerve.getPose().getTranslation().plus(new Translation2d(limelight.getTz() + 1, -limelight.getTx())),
                Rotation2d.fromDegrees(0),
                swerve.getYaw()));

      case 2:
        return PathPlanner.generatePath(
            new PathConstraints(1, 1),

            new PathPoint(
                swerve.getPose().getTranslation(),
                Rotation2d.fromDegrees(0),
                swerve.getYaw()), // Sets starting point of path to current position.

            new PathPoint(
                swerve.getPose().getTranslation().plus(new Translation2d(limelight.getTz() + 0.6, -limelight.getTx())),
                Rotation2d.fromDegrees(0),
                swerve.getYaw()));

      default:
        return new PathPlannerTrajectory();
    }
  }
}