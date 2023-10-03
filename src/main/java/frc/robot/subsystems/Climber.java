// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  TalonFX climberLeft = new TalonFX(12);
  TalonFX climberRight = new TalonFX(13);
  /** Creates a new Climber. */
  public Climber() {
    climberLeft.configForwardSoftLimitEnable(true);
    climberLeft.configReverseSoftLimitEnable(true);
    climberLeft.configForwardSoftLimitThreshold(-10);
    climberLeft.configReverseSoftLimitThreshold(-350766);
    climberRight.configForwardSoftLimitEnable(true);
    climberRight.configReverseSoftLimitEnable(true);
    climberRight.configReverseSoftLimitThreshold(10);
    climberRight.configForwardSoftLimitThreshold(379192);
  }

  public void setPower(double power){
    climberLeft.set(ControlMode.PercentOutput, -power);
    climberRight.set(ControlMode.PercentOutput, power);
  }

  public void setLeftPower(double power){
    climberLeft.set(ControlMode.PercentOutput, -power);
  }

  public void setRightPower(double power){
    climberRight.set(ControlMode.PercentOutput, power);
  }
  
  public void setPower(double leftpower, double rightpower){
    climberLeft.set(ControlMode.PercentOutput, -leftpower);
    climberRight.set(ControlMode.PercentOutput, rightpower);
  }

  public double[] getLengths(){
    return new double[]{climberLeft.getSelectedSensorPosition(),climberRight.getSelectedSensorPosition()};
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
