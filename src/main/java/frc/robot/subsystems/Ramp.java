// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;

public class Ramp extends SubsystemBase {
  DoubleSolenoid rampSolenoid;

  public Ramp() {

    rampSolenoid = new DoubleSolenoid(
      PneumaticsModuleType.CTREPCM, 
      RobotConstants.RampConstants.rampSolenoidREV, 
      RobotConstants.RampConstants.rampSolenoidFWD
    );

    retractRamp();; //Starts retracted.
  }

  public void extendRamp() {
    rampSolenoid.set(DoubleSolenoid.Value.kForward);
  }

  public void retractRamp() {
    rampSolenoid.set(DoubleSolenoid.Value.kReverse);
  }

  public void toggleRamp()
  {
    rampSolenoid.toggle();
  }
}
