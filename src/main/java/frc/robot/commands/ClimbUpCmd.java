// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climber;

public class ClimbUpCmd extends CommandBase {
  private Climber m_climber;
  private AHRS m_gyro;
  private final double LEFTLIMIT = -350000;
  private final double RIGHTLIMIT = 378500;
  private boolean calib = true;
  private boolean isFinishing = false;
  private double MinRoll = Double.MAX_VALUE;
  private boolean isTraversing = false;
  private final double RollThres = 20;
  /** Creates a new ClimbUpCmd. */
  public ClimbUpCmd(Climber climber, AHRS gyro) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_climber = climber;
    m_gyro = gyro;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFinishing = false;
    isTraversing = false;
    MinRoll = Double.MAX_VALUE;
    var lengths = m_climber.getLengths();
    if(lengths[0]>LEFTLIMIT||lengths[1]<RIGHTLIMIT){
      calib = true;
    }else{
      calib = false;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(isFinishing) return;
    if(Math.abs(RobotContainer.m_JoystickLeft.getY())>0.4||Math.abs(RobotContainer.m_JoystickLeft.getX())>0.4||Math.abs(RobotContainer.m_JoystickLeft.getZ())>0.4||RobotContainer.m_JoystickLeft.povDown().getAsBoolean()||RobotContainer.m_JoystickLeft.povUp().getAsBoolean()||RobotContainer.m_JoystickLeft.getHID().getRawButton(2)){
      isFinishing = true;
      return;
    }
    if(calib){
      var lengths = m_climber.getLengths();
      if(lengths[0]>LEFTLIMIT){
        m_climber.setLeftPower(1);
      }else{
        m_climber.setLeftPower(0);
      }
      if(lengths[1]<RIGHTLIMIT){
        m_climber.setRightPower(1);
      }else{
        m_climber.setRightPower(0);
      }
      if(lengths[0]<=LEFTLIMIT&&lengths[1]>=RIGHTLIMIT)
        isFinishing = true;
    }else{
      var cur_rot = m_gyro.getPitch();
      var lengths = m_climber.getLengths();
      if(lengths[1]<=10){
        m_climber.setRightPower(0);
      }else{
        m_climber.setRightPower(-1);
      }
      if(cur_rot<MinRoll)
        MinRoll = cur_rot;
      if(isTraversing){
        m_climber.setLeftPower(-0.8);
      }
      if(Math.abs(cur_rot-MinRoll)>=RollThres){
        isTraversing = true;
        m_climber.setLeftPower(-0.8);
      }
      if(lengths[0]>=-10){
        isFinishing=true;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climber.setPower(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinishing;
  }
}
