// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.BallNiche;
import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.Transit;

public class ShootCmd extends CommandBase {
  private Transit m_transit;
  private ColorSensor m_colorSensor;
  private int isFinishing = 10;
  /** Creates a new ShootCmd. */
  public ShootCmd(Transit transit, ColorSensor colorSensor) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_transit = transit;
    m_colorSensor = colorSensor;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_transit.setShooting(true);
    m_transit.inUse = 10;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_transit.shoot(Transit.ShooterSpeed)){
      m_transit.setShooting(true);
      m_transit.setLiftPower();
    }else{
      m_transit.setShooting(false);
    }
    if(m_colorSensor.getNiches()[0]==BallNiche.EMPTY&&m_colorSensor.getNiches()[1]==BallNiche.EMPTY){
      isFinishing --;
    }else{
      isFinishing = 10;
    }
    m_transit.inUse = 10;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_transit.setShooting(false);
    m_transit.stopLift();
    m_transit.stopShoot();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinishing<=0;
  }
}
