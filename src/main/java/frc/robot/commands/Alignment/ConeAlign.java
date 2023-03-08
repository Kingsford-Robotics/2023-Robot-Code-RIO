// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Alignment;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

public class ConeAlign extends CommandBase {
  /** Creates a new AlignCone. */
  
  Swerve swerve;
  Limelight limelight;
  RobotContainer container;
  
  public ConeAlign(RobotContainer container, Swerve swerve, Limelight limelight) {
    this.swerve = swerve;
    this.limelight = limelight;
    this.container = container;

    addRequirements(swerve, limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    container.setIsArmFront(true);
    swerve.drive(
      new Translation2d(
        Math.abs(limelight.getTz() - 0.7) > 0.05? Math.signum(limelight.getTz() - 0.7) * Math.min(Math.abs((limelight.getTz() - 0.7)) * 50, 0.5): 0, 
        Math.abs(limelight.getTx()) > 0.07? Math.signum(limelight.getTx() * 20) * -Math.min(Math.abs(limelight.getTx() * 20), 0.3): 0
      ), 

      Math.abs(swerve.getYaw().getDegrees()) > 1? -Math.min(swerve.getYaw().getDegrees() * 2, 0.2): 0, 
      false, 
      true
    );
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
