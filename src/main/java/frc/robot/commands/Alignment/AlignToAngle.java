// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Alignment;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class AlignToAngle extends CommandBase {
  /** Creates a new AlignToAngle. */
  
  private Swerve m_Swerve;
  private double angle;

  public AlignToAngle(Swerve m_Swerve, double angle) {
    
    this.m_Swerve = m_Swerve;
    this.angle = angle;

    addRequirements(m_Swerve);
  }


  public double getAngleDifference(double currentAngle, double target)
  {
    if(target - currentAngle < -180)
    {
      return target-currentAngle + 360;
    }

    else{
      return target - currentAngle;
    }

  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Swerve.drive(
      new Translation2d(0, 0), 
      Math.signum(getAngleDifference(m_Swerve.getHeading(), angle)) * Math.min(getAngleDifference(m_Swerve.getHeading(), angle) * 0.1, 0.5), 
      true, 
      false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(angle - m_Swerve.getYaw().getDegrees()) < 2.0;
  }
}
