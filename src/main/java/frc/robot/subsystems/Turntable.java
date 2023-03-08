// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.RobotConstants;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turntable extends SubsystemBase {
  private VictorSPX turntableMotor;

  public Turntable() {
    turntableMotor = new VictorSPX(RobotConstants.Turntable.turntableMotorID);
    turntableMotor.set(ControlMode.PercentOutput, 0.0);
  }

  public void setTurntableSpeed(double speed) {
    turntableMotor.set(ControlMode.PercentOutput, speed);
  }
}