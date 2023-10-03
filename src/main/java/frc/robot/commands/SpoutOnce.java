// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Transit;

public class SpoutOnce extends CommandBase {
  private Transit m_transit;
  private double shooterSpeed_his = 0;
  /** Creates a new SpoutOnce. */
  public SpoutOnce(Transit transit) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_transit = transit;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_transit.setShooting(true);
    if(m_transit.setShooterSpeed==0)
      m_transit.shoot(4000);
    shooterSpeed_his = 0;
    m_transit.inUse = 10;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var curSpeed = m_transit.getShooterSpeed();
    if(m_transit.setShooterSpeed==4000)
      m_transit.shoot(4000);
    var dSpeed = curSpeed-shooterSpeed_his;
    shooterSpeed_his=curSpeed;
    if(Shoot.onlyOnce){
      m_transit.setShooting(false);
      m_transit.stopLift();
    }else if(dSpeed<-1000.0){
      Shoot.onlyOnce = true;
      m_transit.setShooting(false);
      m_transit.stopLift();
    }else{
      m_transit.setShooting(true);
      m_transit.setLiftPower(0,0.3,0.2,1.0);
    }
    m_transit.inUse = 10;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_transit.setShooting(false);
    m_transit.stopLift();
    m_transit.stopShoot();
    Shoot.onlyOnce = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
