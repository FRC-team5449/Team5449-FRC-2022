// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.BallNiche;
import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.Transit;

public class BallScreeningCmd extends CommandBase {
  private ColorSensor m_colorSensor;
  private Transit m_transit;
  private Alliance m_alliance;
  /** Creates a new BallScreeningCmd. */
  public BallScreeningCmd(ColorSensor colorSensor, Transit transit) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_colorSensor = colorSensor;
    m_transit = transit;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_alliance = DriverStation.getAlliance();
  }

  private void detrude(){
    m_transit.setShooting(false);
    m_transit.stopShoot();
    m_transit.setIntaking(true);
    m_transit.setLiftPower(-0.9,-0.9,-0.2,-0.9);
  }

  private void spout(){
    m_transit.stopLift();
    m_transit.setIntaking(false);
    m_transit.shoot(4000);
    m_transit.setShooting(true);
    m_transit.setLiftPower(0,0.7,0.3,1.0);
  }

  private void stop(){
    m_transit.setShooting(false);
    m_transit.setIntaking(false);
    m_transit.stopLift();
    m_transit.stopShoot();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_transit.inUse > 0){
      m_transit.inUse --;
      return;
    }
    Constants.BallNiche[] niches = m_colorSensor.getNiches();
    switch(niches[0]){
      case BLUE:
        if(m_alliance==Alliance.Red){
          detrude();
        }else if(m_alliance==Alliance.Blue){
          switch(niches[1]){
            case BLUE:
              stop();
              break;
            case RED:
              spout();
              break;
            case EMPTY:
              stop();
              break;
          }
        }
        break;
      case RED:
        if(m_alliance==Alliance.Blue){
          detrude();
        }else if(m_alliance==Alliance.Red){
          switch(niches[1]){
            case BLUE:
              spout();
              break;
            case RED:
              stop();
              break;
            case EMPTY:
              stop();
              break;
          }
        }
        break;
      case EMPTY:
        if(m_alliance==Alliance.Blue&&niches[1]==BallNiche.RED){
          detrude();
        }else if(m_alliance==Alliance.Red&&niches[1]==BallNiche.BLUE){
          detrude();
        }
        else{
          stop();
        }
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
