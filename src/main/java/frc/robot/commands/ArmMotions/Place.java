// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmMotions;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

public class Place {
    Arm arm;
    Elevator elevator;
    RobotContainer robotContainer;

    public Place(RobotContainer robotContainer, Arm arm, Elevator elevator) {
        this.robotContainer = robotContainer;
        this.arm = arm;
        this.elevator = elevator;
    }

    public SequentialCommandGroup getCommand()
    {
        List<CommandBase> commandList = new ArrayList<CommandBase>();
        SequentialCommandGroup group;

        commandList.add(
            new InstantCommand(() -> elevator.setElevatorHeight(15.4, 0.7), elevator)
        );

        commandList.add(
            new WaitUntilCommand(() -> elevator.isElevatorToPosition())
        );

        commandList.add(
            new InstantCommand(() -> arm.retract(), arm)
        );

        commandList.add(
            new WaitCommand(0.35)
        );

        commandList.add(
            new InstantCommand(() -> arm.setArmAngle(80, 0.8), arm)
        );

        commandList.add(
            new WaitUntilCommand(() -> arm.getAngle().getDegrees() < 85)
        );

        commandList.add(
            new ConditionalCommand(
                new InstantCommand(() -> arm.setArmAngle(-3.0, 1.0), arm),
                new ConditionalCommand(
                    new InstantCommand(() -> arm.setArmAngle(10, 1.0), arm),
                    new InstantCommand(() -> arm.setArmAngle(30, 1.0), arm),
                    () -> robotContainer.getLevel() == 1
                ),

                () -> robotContainer.getLevel() == 2
            )
        );

        commandList.add(
            new WaitUntilCommand(() -> arm.getAngle().getDegrees() < 80)
        );

        commandList.add(
            new ConditionalCommand(
                new InstantCommand(() -> elevator.setElevatorHeight(10, 0.6), elevator), 
                new ConditionalCommand(
                    new InstantCommand(() -> elevator.setElevatorHeight(10, 0.6), elevator), 
                    new InstantCommand(() -> elevator.setElevatorHeight(1.0, 0.6), elevator), 
                    () -> robotContainer.getLevel() == 1), 
                () -> robotContainer.getLevel() == 2
        ));

        commandList.add(new WaitUntilCommand(() -> elevator.isElevatorToPosition()));

        commandList.add(
            new WaitUntilCommand(() -> arm.isArmToPosition())
        );

        commandList.add(
            new ConditionalCommand(
                new InstantCommand(() -> arm.extend(), arm), 
                new InstantCommand(() -> arm.retract(), arm), 
                () -> robotContainer.getLevel() == 2 || robotContainer.getLevel() == 1
            )
        );

        group = new SequentialCommandGroup(commandList.toArray(new CommandBase[commandList.size()]));
        return group;
    }
}