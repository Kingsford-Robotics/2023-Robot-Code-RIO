// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

/** Add your docs here. */
public class GroundGrab {
    Arm arm;
    Elevator elevator;

    public GroundGrab(Arm arm, Elevator elevator) {
        this.arm = arm;
        this.elevator = elevator;
    }

    public SequentialCommandGroup getCommand()
    {
        List<CommandBase> commandList = new ArrayList<CommandBase>();
        SequentialCommandGroup group;

        commandList.add(new InstantCommand(() -> arm.retract(), arm));

        commandList.add(new InstantCommand(() -> arm.open(), arm));

        commandList.add(new InstantCommand(() -> elevator.setElevatorHeight(10.0, 0.2), elevator));

        commandList.add(new WaitUntilCommand(() -> elevator.isElevatorToPosition()));

        commandList.add(new InstantCommand(() -> arm.setArmAngle(73.0, 0.2), arm));

        commandList.add(new InstantCommand(() -> arm.open(), arm));

        commandList.add(new WaitUntilCommand(() -> arm.isArmToPosition()));

        commandList.add(new InstantCommand(() -> arm.extend(), arm));

        group = new SequentialCommandGroup(commandList.toArray(new CommandBase[commandList.size()]));
        return group;
    }
}