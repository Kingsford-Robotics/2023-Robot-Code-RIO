// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Alignment;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

public class PreciseAlign extends CommandBase {
  /** Creates a new PreciseAlign. */

  private Swerve swerve;
  private Limelight limelight;
  
  public PreciseAlign(Swerve swerve, Limelight limelight) {
    this.swerve = swerve;
    this.limelight = limelight;

    addRequirements(
      swerve,
      limelight
    );
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double theta = Math.signum(limelight.getAngle()) * Math.min(Math.abs(limelight.getAngle() * 0.1), 0.2);

    swerve.drive(
      new Translation2d(0.0, 0.0), 
      theta, 
      false, 
      true
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(limelight.getTx()) < 0.5;
  }
}
