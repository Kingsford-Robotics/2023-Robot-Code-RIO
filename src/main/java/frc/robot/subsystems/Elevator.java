// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;


//TODO: Consider adding top calibration back in.
public class Elevator extends SubsystemBase {
  /*Creates new Elevator */
  private TalonFX elevatorMotor;
  
  //Wired normally closed (false = pressed)
  private DigitalInput topLimitSwitch;
  private DigitalInput bottomLimitSwitch;

  /*Shuffleboard Setup*/
  private ShuffleboardTab elevatorTab;
  
  private GenericEntry elevatorHeight;
  private GenericEntry elevatorEncoder;
  private GenericEntry elevatorSpeed;

  private GenericEntry elevatorTopLimitSwitch;
  private GenericEntry elevatorBottomLimitSwitch;

  private GenericEntry isToPosition;
  private GenericEntry targetEncoderPosition;

  public Elevator(double initialPosition) {
    elevatorMotor = new TalonFX(RobotConstants.ElevatorConstants.elevatorMotorID);
    elevatorMotor.configFactoryDefault();
    elevatorMotor.setNeutralMode(com.ctre.phoenix.motorcontrol.NeutralMode.Brake);
    elevatorMotor.setInverted(true);
    //Set motor PID constants
    elevatorMotor.config_kP(0, RobotConstants.ElevatorConstants.elevatorKp);
    elevatorMotor.config_kI(0, RobotConstants.ElevatorConstants.elevatorKi);
    elevatorMotor.config_kD(0, RobotConstants.ElevatorConstants.elevatorKd);
    elevatorMotor.config_kF(0, RobotConstants.ElevatorConstants.elevatorKF);

    elevatorMotor.configAllowableClosedloopError(0, 0.15 / RobotConstants.ElevatorConstants.elevatorTravelEncoderTick);
    //Configure Motion Magic
    elevatorMotor.configMotionCruiseVelocity(RobotConstants.ElevatorConstants.elevatorCruiseVelocity);
    elevatorMotor.configMotionAcceleration(RobotConstants.ElevatorConstants.elevatorMaxAcceleration);
    
    //Acceleration smoothing
    elevatorMotor.configMotionSCurveStrength(RobotConstants.ElevatorConstants.elevatorSCurveStrength);

    topLimitSwitch = new DigitalInput(RobotConstants.ElevatorConstants.elevatorTopLimitSwitchID);
    bottomLimitSwitch = new DigitalInput(RobotConstants.ElevatorConstants.elevatorBottomLimitSwitchID);

    calibrateElevator(initialPosition);

    /*Shuffleboard Setup*/
    elevatorTab = Shuffleboard.getTab("Elevator");

    elevatorHeight = elevatorTab.add("Elevator Height", 0.0).getEntry();
    elevatorEncoder = elevatorTab.add("Elevator Encoder", 0.0).getEntry();
    elevatorSpeed = elevatorTab.add("Elevator Speed", 0.0).getEntry();

    elevatorTopLimitSwitch = elevatorTab.add("Top Limit Switch", false).getEntry();
    elevatorBottomLimitSwitch = elevatorTab.add("Bottom Limit Switch", false).getEntry();

    isToPosition = elevatorTab.add("Is To Position", false).getEntry();
    targetEncoderPosition = elevatorTab.add("Target Encoder Position", 0.0).getEntry();
  }

  /**
   * {@summary} Use this function to calibrate the elevator position when it is at a known height relative to the lowest position.
   * @param currentHeight The height to set the elevator encoder at.s
   */
  public void calibrateElevator(double currentHeight)
  {
    double encoderPosition = currentHeight / RobotConstants.ElevatorConstants.elevatorTravelEncoderTick;
    elevatorMotor.setSelectedSensorPosition(encoderPosition);
  }

  /**
   * {@summary} Sets the elevator motor percent output.
   * @param speed Elevator motor percent speed. -1.0 to 1.0.
   */
  public void setElevatorSpeed(double speed){
    //Set speed to 0 if touching limit switch and moving towards it.
    if((getTopLimitSwitch() || getElevatorPosition() > RobotConstants.ElevatorConstants.elevatorMaxTravel - RobotConstants.ElevatorConstants.safeZone) && speed > 0)
    {
      speed = 0;
    }
    else if(getBottomLimitSwitch() && speed < 0)
    {
      speed = 0;
    }

    elevatorMotor.set(ControlMode.PercentOutput, speed);
  }

  /**
   * {@summary} Sets the elevator to the height specified using a S motion curve.
   * @param height Height, in inches, above the lowest elevator position.
   */
  public void setElevatorHeight(double height, double percentSpeed){
    elevatorMotor.configMotionCruiseVelocity(RobotConstants.ElevatorConstants.elevatorCruiseVelocity * percentSpeed);

    if(height > RobotConstants.ElevatorConstants.elevatorMaxTravel - RobotConstants.ElevatorConstants.safeZone)
    {
      height = RobotConstants.ElevatorConstants.elevatorMaxTravel - RobotConstants.ElevatorConstants.safeZone;
    }
    else if(height < RobotConstants.ElevatorConstants.safeZone)
    {
      height = RobotConstants.ElevatorConstants.safeZone;
    }

    double encoderPosition = height / RobotConstants.ElevatorConstants.elevatorTravelEncoderTick;

    targetEncoderPosition.setDouble(encoderPosition);

    elevatorMotor.set(ControlMode.MotionMagic, encoderPosition);
  }

  public void setElevatorHeight(double height){
    setElevatorHeight(height, 1.0);
  }

  public boolean isElevatorToPosition() {
    if(Math.abs(targetEncoderPosition.getDouble(0) - elevatorMotor.getSelectedSensorPosition()) < 0.15 / RobotConstants.ElevatorConstants.elevatorTravelEncoderTick)
    {
      return true;
    }

    return false;
  }

  public boolean getTopLimitSwitch(){
    return topLimitSwitch.get();
  }

  public boolean getBottomLimitSwitch(){
    return bottomLimitSwitch.get();
  }

  //Returns elevator height in meters relative to lowest position.
  public double getElevatorPosition(){
    return elevatorMotor.getSelectedSensorPosition() * RobotConstants.ElevatorConstants.elevatorTravelEncoderTick;
  } 

  public double getElevatorEncoder()
  {
    return elevatorMotor.getSelectedSensorPosition();
  }

  @Override
  public void periodic() {
    if(getTopLimitSwitch())
    {
      if(elevatorMotor.getMotorOutputPercent() > 0)
      {
        elevatorMotor.set(ControlMode.PercentOutput, 0);
      }
    }

    if(getBottomLimitSwitch())
    {
      calibrateElevator(0.0);

      if(elevatorMotor.getMotorOutputPercent() < 0)
      {
        elevatorMotor.set(ControlMode.PercentOutput, 0);
      }
    }

    elevatorHeight.setDouble(getElevatorPosition());
    elevatorEncoder.setDouble(getElevatorEncoder());
    elevatorSpeed.setDouble(elevatorMotor.getMotorOutputPercent());

    elevatorTopLimitSwitch.setBoolean(getTopLimitSwitch());
    elevatorBottomLimitSwitch.setBoolean(getBottomLimitSwitch());

    isToPosition.setBoolean(isElevatorToPosition());
  }

public Object isAtHeight(double d) {
    return null;
}
}
