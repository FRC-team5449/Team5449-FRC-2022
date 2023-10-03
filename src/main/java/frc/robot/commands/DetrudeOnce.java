// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.BallNiche;
import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.Transit;

public class DetrudeOnce extends CommandBase {
  private Transit m_transit;
  private ColorSensor m_colorSensor;
  Constants.BallNiche[] m_niches;
  /** Creates a new DetrudeOnce. */
  public DetrudeOnce(Transit transit, ColorSensor colorSensor) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_transit = transit;
    m_colorSensor = colorSensor;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_niches = m_colorSensor.getNiches();
    m_transit.setIntaking(true);
    m_transit.inUse = 10;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Constants.BallNiche[] niches = m_colorSensor.getNiches();
    if(m_niches[0]==BallNiche.EMPTY){
      if(niches[1]==BallNiche.EMPTY&&niches[0]==BallNiche.EMPTY){
        m_transit.stopLift();
      }else{
        m_transit.setLiftPower(-0.9,-0.9,-0.2,-0.9);
      }
    }else if(niches[0]!=m_niches[0]||niches[1]!=m_niches[1]){
      m_transit.stopLift();
    }else{
      m_transit.setLiftPower(-0.9,-0.9,-0.2,-0.9);
    }
    m_transit.inUse = 10;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_transit.stopLift();
    m_transit.setIntaking(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
