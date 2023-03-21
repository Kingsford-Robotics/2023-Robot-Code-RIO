// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class LevelChargeStation extends CommandBase {
  /** Creates a new LevelChargeStation. */
  
  Swerve m_Swerve;
  boolean hasTitled;
  boolean secondTilt;
  double startLevelTime;

  double levelTime = 1.0;

  double xOutput;

  //TODO: Add rotation correction.

  public LevelChargeStation(Swerve swerve) {
    this.m_Swerve = swerve;
    startLevelTime = Timer.getFPGATimestamp();
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
          new Translation2d(-1.5, 0),
          0,
          false,
          true
      );

      if(m_Swerve.getTilt() < -5)
      {
        hasTitled = true;
      }
    }

    else{ 
      xOutput = Math.signum(m_Swerve.getTilt()) * Math.min(Math.abs(m_Swerve.getTilt()) * (secondTilt? 0.05: 0.3), 0.6);
       
      m_Swerve.drive(
          new Translation2d(xOutput, 0),
          0,
          false,
          true  
      );

      if(m_Swerve.getTilt() > 0.75)
      {
        secondTilt = true;
        System.out.println("Second Tilt Finished!!!!");
      }

      if(Math.abs(m_Swerve.getTilt()) > 0.5)
      {
        startLevelTime = Timer.getFPGATimestamp();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return hasTitled && (Timer.getFPGATimestamp() - startLevelTime > levelTime);
  }
}