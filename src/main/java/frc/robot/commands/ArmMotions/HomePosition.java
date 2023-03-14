// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmMotions;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

/** Add your docs here. */
public class HomePosition {
    Arm arm;
    Elevator elevator;

    public HomePosition(Arm arm, Elevator elevator) {
        this.arm = arm;
        this.elevator = elevator;
    }

    public SequentialCommandGroup getCommand() {
        List<CommandBase> commandList = new ArrayList<CommandBase>();

        SequentialCommandGroup group;

        commandList.add(
            new InstantCommand(() -> arm.retract(), arm))
        ;

        commandList.add(
            new InstantCommand(() -> arm.open(), elevator)
        );
        
        commandList.add(
            new InstantCommand(() -> elevator.setElevatorHeight(7.5, 0.6), elevator)
        );

        commandList.add(
            new WaitUntilCommand(() -> elevator.isElevatorToPosition())
        );

        commandList.add(
            new InstantCommand(() -> arm.setArmAngle(105, 0.8), arm)
        );

        commandList.add(
            new WaitUntilCommand(() -> arm.isArmToPosition())
        );

        commandList.add(
            new InstantCommand(() -> elevator.setElevatorHeight(5.5, 0.5), elevator)
        );

        commandList.add(
            new InstantCommand(() -> arm.setArmAngle(99.0, 0.3), arm)
        );
            
        commandList.add(
            new WaitUntilCommand(() -> arm.isArmToPosition()) 
        );

        commandList.add(
            new WaitUntilCommand(() -> elevator.isElevatorToPosition())
        );

        commandList.add(
            new WaitCommand(0.25)
        );

        commandList.add(
            new InstantCommand(() -> elevator.setElevatorHeight(3.85, 0.2), elevator)
        );

        commandList.add(
            new WaitUntilCommand(() -> elevator.isElevatorToPosition())
        );

        //Create sequential command group from list
        group = new SequentialCommandGroup(commandList.toArray(new CommandBase[commandList.size()]));
        return group;
    }
}