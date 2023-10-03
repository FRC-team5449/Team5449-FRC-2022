// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Transit extends SubsystemBase {
  
  TalonSRX intakeC = new TalonSRX(Constants.Intake3);
  TalonSRX intakeB = new TalonSRX(Constants.Intake2);
  TalonSRX intakeD = new TalonSRX(Constants.Intake4);
  TalonFX shooter = new TalonFX(Constants.Shooter);
  // TalonFX intakeA = new TalonFX(Constants.Intake1);
  CANSparkMax intakeA = new CANSparkMax(Constants.Intake1, MotorType.kBrushless);
  Solenoid arm = new Solenoid(Constants.PneumaticHub, PneumaticsModuleType.REVPH, 4);
  private boolean solenoid_down = true;
  private boolean isShooting = false;
  private boolean isIntaking = false;
  private double shooterSpeed_his = 0;
  private boolean dSpeeddescend = false;
  public double setShooterSpeed = 0;
  public int inUse = 0;
  public static double ShooterSpeed = Constants.ShooterMaxSpeed;
  public static int ballsShooted = 0;

  /** Creates a new Transit. */
  public Transit() {
    intakeD.configVoltageCompSaturation(12);
    intakeD.enableVoltageCompensation(true);
    shooter.configVoltageCompSaturation(12);
    shooter.enableVoltageCompensation(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    var curSpeed = getShooterSpeed();
    var dSpeed = curSpeed-shooterSpeed_his;
    if(dSpeed<-1000.0&&!dSpeeddescend){
      ballsShooted++;
      dSpeeddescend = true;
    }else if(dSpeed>-1000.0){
      dSpeeddescend = false;
    }
    shooterSpeed_his = curSpeed;
  }

  public boolean shoot(double speed) {
    setShooterSpeed = speed;
    shooter.set(ControlMode.Velocity, speed);
    var curSpeed = shooter.getSelectedSensorVelocity();
    return Math.abs((speed-curSpeed))<Constants.ShooterSpeedError;
  }

  public void stopShoot() {
    setShooterSpeed = 0;
    shooter.set(ControlMode.PercentOutput, 0);
  }

  public void setLiftPower(){
    if(isIntaking){
      if(solenoid_down){
        intakeA.set(-1.0);
      }else{
        intakeA.set(0);
      }
      intakeB.set(ControlMode.PercentOutput, -1.0);
    }
    intakeC.set(ControlMode.PercentOutput, -1.0);
    if(isShooting&&!isIntaking)
      intakeD.set(ControlMode.PercentOutput, 0.35);
    else
      intakeD.set(ControlMode.PercentOutput, -0.4);
  }

  public void setLiftPower(double power){
    if(isIntaking){
      if(solenoid_down){
        intakeA.set(-power);
      }else{
        intakeA.set(0);
      }
      intakeB.set(ControlMode.PercentOutput, -power);
    }
    intakeC.set(ControlMode.PercentOutput, -power);
    if(isShooting&&!isIntaking)
      intakeD.set(ControlMode.PercentOutput, power);
    else
      intakeD.set(ControlMode.PercentOutput, -power);
  }

  public void setLiftPower(double powerA, double powerB, double powerC, double powerD){
    if(isIntaking){
      if(solenoid_down){
        intakeA.set(-powerA);
      }else{
        intakeA.set(0);
      }
      intakeB.set(ControlMode.PercentOutput, -powerB);
    }
    intakeC.set(ControlMode.PercentOutput, -powerC);
    intakeD.set(ControlMode.PercentOutput, powerD);
  }

  public void stopLift(){
    intakeB.set(ControlMode.PercentOutput, 0.0);
    intakeC.set(ControlMode.PercentOutput, 0.0);
    intakeD.set(ControlMode.PercentOutput, 0.0);
    intakeA.set(0);
  }

  public void setShooting(boolean isShooting) {
    this.isShooting = isShooting;
  }

  public void setIntaking(boolean isIntaking) {
    this.isIntaking = isIntaking;
  }

  public void armDown(){
    arm.set(true);
    solenoid_down = true;
  }

  public void armUp(){
    arm.set(false);
    solenoid_down = false;
  }

  public double getShooterSpeed(){
    return shooter.getSelectedSensorVelocity();
  }
}
