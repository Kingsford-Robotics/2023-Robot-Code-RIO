// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.networktables.BooleanArraySubscriber;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class JetsonXavier extends SubsystemBase {
  
  public enum targetType
  {
    CONE,
    CUBE
  }

  public class targetInfo
  {
    public double[] position;
    public boolean trackingStatus;
    public targetType type;
    public double confidence;
  }
  
  //Check arraylist syntax.
  private ArrayList<targetInfo> targets;
  private double[] robotPose;
  private boolean isTracking;

  NetworkTableInstance inst;
  NetworkTable visionTable;

  DoubleArraySubscriber positionsSubscriber;
  BooleanArraySubscriber trackingStatusSubscriber;
  BooleanArraySubscriber targetTypeSubscriber;
  DoubleArraySubscriber confidenceSubscriber;

  DoubleArraySubscriber robotPoseSubscriber;
  BooleanSubscriber isTrackingSubscriber;

  BooleanPublisher turnOffJetson;
  /** Creates a new Jetson Xavier. */
  public JetsonXavier() {
    targets = new ArrayList<targetInfo>();
    robotPose = new double[3];
    isTracking = false;

    inst = NetworkTableInstance.getDefault();
    visionTable = inst.getTable("Vision");

    positionsSubscriber = visionTable.getDoubleArrayTopic("positions").subscribe(new double[] {}, PubSubOption.keepDuplicates(true));
    trackingStatusSubscriber = visionTable.getBooleanArrayTopic("trackingStatus").subscribe(new boolean[] {}, PubSubOption.keepDuplicates(true));
    targetTypeSubscriber = visionTable.getBooleanArrayTopic("targetType").subscribe(new boolean[] {}, PubSubOption.keepDuplicates(true));
    confidenceSubscriber = visionTable.getDoubleArrayTopic("confidence").subscribe(new double[] {}, PubSubOption.keepDuplicates(true));

    robotPoseSubscriber = visionTable.getDoubleArrayTopic("robotPose").subscribe(new double[] {}, PubSubOption.keepDuplicates(true));
    isTrackingSubscriber = visionTable.getBooleanTopic("isTracking").subscribe(false, PubSubOption.keepDuplicates(true));

    turnOffJetson = visionTable.getBooleanTopic("turnOffJetson").publish(PubSubOption.keepDuplicates(true));
  }

  public void updateTargets()
  {
    targets.clear();
    double[] positions = positionsSubscriber.get();
    boolean[] trackingStatus = trackingStatusSubscriber.get();
    boolean[] targetType = targetTypeSubscriber.get();
    double[] confidence = confidenceSubscriber.get();

    for(int i = 0; i < targetType.length; i++)
    {
      targetInfo target = new targetInfo();

      target.position = new double[3];
      target.position[0] = positions[i * 3 + 0];
      target.position[1] = positions[i * 3 + 1];
      target.position[2] = positions[i * 3 + 2];

      target.trackingStatus = trackingStatus[i];
      target.type = targetType[i] ? JetsonXavier.targetType.CONE : JetsonXavier.targetType.CUBE;
      target.confidence = confidence[i];
      targets.add(target);
    }
  }

  public ArrayList<targetInfo> getTargets()
  {
    return targets;
  }

  public double[] getRobotPose()
  {
    return robotPose;
  }

  public boolean getIsTracking()
  {
    return isTracking;
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateTargets();
    robotPose = robotPoseSubscriber.get();
    isTracking = isTrackingSubscriber.get();
  }
}