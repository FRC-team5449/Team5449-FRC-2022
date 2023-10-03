// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimeLight extends SubsystemBase {
  private NetworkTable mNetworkTable;
  private DoubleSubscriber tx;
  /** Creates a new LimeLight. */
  public LimeLight() {
    mNetworkTable = NetworkTableInstance.getDefault().getTable("limelight");
    tx = mNetworkTable.getDoubleTopic("tx").subscribe(0.0);
  }

  public double getHorizontalOffset(){
    return tx.get();
  }

  public void turnOnLed(){
    mNetworkTable.getEntry("ledMode").setNumber(3);
  }

  public void turnOffLed(){
    mNetworkTable.getEntry("ledMode").setNumber(1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
