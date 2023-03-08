// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

public class StopArmElevator extends InstantCommand {
  
  public Arm arm;
  public Elevator elevator;

  public StopArmElevator(Arm arm, Elevator elevator) {
    this.arm = arm;
    this.elevator = elevator;

    addRequirements(arm, elevator);
  }

  // Called when the command is initially scheduled.
  @Override

  public void initialize() {
    arm.setArmSpeed(0.0);
    elevator.setElevatorSpeed(0.0);
  }
}
