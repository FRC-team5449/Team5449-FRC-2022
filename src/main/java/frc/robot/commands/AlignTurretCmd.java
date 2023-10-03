// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LimeLight;

public class AlignTurretCmd extends CommandBase {
  private DriveTrain m_driveTrain;
  private LimeLight m_limeLight;
  private final double Kp = 0.1;
  /** Creates a new AlignTurretCmd. */
  public AlignTurretCmd(DriveTrain driveTrain, LimeLight limeLight) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driveTrain = driveTrain;
    m_limeLight = limeLight;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double tx = m_limeLight.getHorizontalOffset();
    if(tx>8.0){
      m_driveTrain.drive(-1.8);
    }else if(tx<-8.0){
      m_driveTrain.drive(1.8);
    }else{
      m_driveTrain.drive(-tx*Kp);
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
