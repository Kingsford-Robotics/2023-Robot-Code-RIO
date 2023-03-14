// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Alignment;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

public class PreciseAlign extends CommandBase {
  /** Creates a new PreciseAlign. */

  private Swerve swerve;
  private Limelight limelight;

  private PIDController thetaController;
  
  public PreciseAlign(Swerve swerve, Limelight limelight) {
    this.swerve = swerve;
    this.limelight = limelight;

    thetaController = new PIDController(0.5, 0, 0);

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

    double theta = thetaController.calculate(limelight.getTx(), 0);

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
