// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class LevelChargeStation extends CommandBase {
  /** Creates a new LevelChargeStation. */
  
  Swerve m_Swerve;
  boolean isFromBack;
  boolean hasTitled;

  PIDController level;
  double xOutput;

  //TODO: Add rotation correction.

  public LevelChargeStation(Swerve swerve, boolean isFromBack) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_Swerve = swerve;
    this.isFromBack = isFromBack;

    level = new PIDController(0.3, 0, 0);

    addRequirements(m_Swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!hasTitled){
      m_Swerve.drive(
          new Translation2d(isFromBack? -0.5: 0.5, 0),
          0,
          false,
          true
      );

      if(Math.abs(m_Swerve.getTilt()) > 15)
      {
        hasTitled = true;
      }
    }

    else{ 
      xOutput = level.calculate(m_Swerve.getTilt(), 0);
      
      m_Swerve.drive(
          new Translation2d(xOutput, 0),
          0,
          false,
          true  
      );
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return hasTitled && Math.abs(m_Swerve.getTilt()) < 2;
  }
}