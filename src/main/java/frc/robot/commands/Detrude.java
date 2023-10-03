// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Transit;

public class Detrude extends CommandBase {
  private Transit m_transit;
  /** Creates a new Detrude. */
  public Detrude(Transit transit) {
    // Use addRequirements() here to declare subsystem dependencies.
    // addRequirements(transit);
    m_transit = transit;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_transit.setIntaking(true);
    m_transit.inUse = 10;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_transit.setLiftPower(-0.9,-0.9,-0.9,-0.9);
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
