// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode.PixelFormat;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class DashboardDisplay extends SubsystemBase {
  
  /*Subsystem References for Data and Commands*/
  private RobotContainer m_RobotContainer;
  private Swerve m_Swerve;
  private Arm m_arm;

  private Limelight m_Limelight;

  private MjpegServer turntableCameraServ;
  private MjpegServer frontCameraServ;

  /*Camera Streams*/
  private UsbCamera turnTableCamera;
  private UsbCamera frontCamera;

  /*Shuffleboard Tab*/
  private ShuffleboardTab competitionTab;

  /*Shuffleboard Data*/
  private GenericEntry gyroAngle;
  private GenericEntry targetType;
  private GenericEntry scoreLocation;
  private GenericEntry isArmFront;;

  private GenericEntry heading;

  private GenericEntry alignment;

  private GenericEntry autoAlignOn;

  //private GenericEntry targetDistance;
  
  private Field2d field = new Field2d();

  private SendableChooser mChooser;

  public DashboardDisplay(RobotContainer m_RobotContainer, Swerve m_Swerve, Arm m_arm, Limelight m_Limelight) {
    /*Subsystem Instantiation */
    this.m_RobotContainer = m_RobotContainer;
    this.m_Swerve = m_Swerve;
    this.m_arm = m_arm;
    this.m_Limelight = m_Limelight;

    competitionTab = Shuffleboard.getTab("Competition");

    /*Camera Instantiation*/
    turnTableCamera = CameraServer.startAutomaticCapture(0);
    frontCamera = CameraServer.startAutomaticCapture(1);

    turnTableCamera.setVideoMode(PixelFormat.kMJPEG, 480, 320, 10);
    frontCamera.setVideoMode(PixelFormat.kMJPEG, 480, 320, 10);

    turntableCameraServ = new MjpegServer("Turntable", 1182);
    frontCameraServ = new MjpegServer("Front", 1183);

    turntableCameraServ.setSource(turnTableCamera);
    frontCameraServ.setSource(frontCamera);

    //Add compression to the cameras
    turntableCameraServ.getProperty("compression").set(30);
    frontCameraServ.getProperty("compression").set(30);

    competitionTab.add("Turntable", turntableCameraServ.getSource());
    competitionTab.add("Front",frontCameraServ.getSource());


    /*Shuffleboard Data Instantiation*/
    gyroAngle = competitionTab.add("Gyro Angle", 0).getEntry();
    competitionTab.add(field);

    targetType = competitionTab.add("Target Type", false).getEntry();
    scoreLocation = competitionTab.add("Score Location", "Unknown").getEntry();
    isArmFront = competitionTab.add("Arm Front", false).getEntry();

    alignment = competitionTab.add("Alignment", "Guide Right").getEntry();

    autoAlignOn = competitionTab.add("Auto Align On", true).getEntry();

    competitionTab.add("Toggle Arm", new InstantCommand(() -> m_arm.toggleExtension()));
  }

  @Override
  public void periodic() {
    field.setRobotPose(m_Swerve.getPose());
    gyroAngle.setDouble(m_Swerve.getYaw().getDegrees());

    targetType.setBoolean(m_RobotContainer.getIsCone());

    isArmFront.setBoolean(m_RobotContainer.getIsFrontArm());
    
    switch(m_RobotContainer.getLevel()){
      case 0:
        scoreLocation.setString("GROUND");
        break;
      case 1:
        scoreLocation.setString("MID");
        break;
      case 2:
        scoreLocation.setString("HIGH");
        break;
      default:
        scoreLocation.setString("Unknown");
        break;
    }

    if(m_RobotContainer.isAlignRight())
    {
      alignment.setString("Guide Right");
    }

    else{
      alignment.setString("Guide Left");
    }

    m_RobotContainer.setAutoAlign(autoAlignOn.getBoolean(false));
  }
}