// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


/**
 * !!! Deconstructed
 * 2023/4/11
 */
public class DifferentialModel extends SubsystemBase {

  TalonFX motor1;
  TalonFX motor2;
  final Constants.DifferentialModelType modelType;

  /** Creates a new DifferentialModel. */
  public DifferentialModel(Constants.DifferentialModelType modelType) {
    switch(modelType){
      case LEFTMODEL:
        motor1 = new TalonFX(1);
        motor2 = new TalonFX(2);
        break;
      case RIGHTMODEL:
        motor1 = new TalonFX(4);
        motor2 = new TalonFX(3);
        break;
    }
    this.modelType = modelType;
  }

  public void setPower(double power){
    switch(modelType){
      case LEFTMODEL:
        motor1.set(ControlMode.PercentOutput, power);
        motor2.set(ControlMode.PercentOutput, power);
        break;
      case RIGHTMODEL:
        motor1.set(ControlMode.PercentOutput, -power);
        motor2.set(ControlMode.PercentOutput, -power);
        break;
    }
  }

  public double getPower(){
    switch(modelType){
      case LEFTMODEL:
        return (motor1.getMotorOutputPercent()+motor2.getMotorOutputPercent())/2.0;
      case RIGHTMODEL:
        return -(motor1.getMotorOutputPercent()+motor2.getMotorOutputPercent())/2.0;
      default:
        return 0.0;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
