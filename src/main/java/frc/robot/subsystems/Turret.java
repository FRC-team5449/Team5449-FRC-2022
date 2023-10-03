// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Turret extends SubsystemBase {
  TalonSRX turretAngle = new TalonSRX(Constants.turretAngle);
  double last_current = 0;
  // TalonSRX turretDirection = new TalonSRX(Constants.turretDirection);    // Deconstruction
  /** Creates a new Turret. */
  public Turret() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void adjustAngle(double power){
    turretAngle.set(ControlMode.PercentOutput, power);
    if(power==0)
      turretAngle.set(ControlMode.Position, turretAngle.getSelectedSensorPosition());
  }

  public void setAngle(double angle){
    turretAngle.set(ControlMode.Position, (Constants.MaxTurretAngle-angle)/Math.abs(Constants.MaxTurretAngle-Constants.MinTurretAngle)*Constants.TurretAnglePulseRange);
  }

  // public void adjustDirection(double power){
  //   turretDirection.set(ControlMode.PercentOutput, power);
  //   if(power==0)
  //     turretAngle.set(ControlMode.PercentOutput, 0);
  // }

  public double getAngle(){
    return Constants.MaxTurretAngle-(turretAngle.getSelectedSensorPosition()/Constants.TurretAnglePulseRange)*Math.abs(Constants.MaxTurretAngle-Constants.MinTurretAngle);
  }
}
