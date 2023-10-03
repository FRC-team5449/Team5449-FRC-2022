// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climber;

public class ClimbUp extends CommandBase {
  private Climber m_climber;
  private ClimbUpCmd m_climbUpCmd;
  public static boolean async_control = false;
  /** Creates a new ClimbUp. */
  public ClimbUp(Climber climber, ClimbUpCmd climbUpCmd) {
    // Use addRequirements() here to declare subsystem dependencies.
    // addRequirements(climber);
    m_climber = climber;
    m_climbUpCmd = climbUpCmd;
    SmartDashboard.putString("Sub Mode", "S");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_climbUpCmd.isScheduled())  return;
    if(RobotContainer.m_driveMode==Constants.DriveMode.CLIMBUP){
      if(async_control){
        var leftpower = -MathUtil.applyDeadband(RobotContainer.m_JoystickLeft.getY(), 0.2);
        var rightpower = -MathUtil.applyDeadband(RobotContainer.m_JoystickRight.getY(), 0.2);
        m_climber.setPower(rightpower, leftpower);
      }else{
        var power = -MathUtil.applyDeadband(RobotContainer.m_JoystickLeft.getY(), 0.2);
        m_climber.setRightPower(power);
      }
      switch(RobotContainer.m_JoystickLeft.getHID().getPOV()){
        case 0:
          m_climber.setLeftPower(0.8);
          break;
        case 180:
          m_climber.setLeftPower(-0.8);
          break;
        default:
          m_climber.setLeftPower(0);
          break;
      }
    }else{
      m_climber.setPower(0, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  public void setAsync(boolean async_control){
    ClimbUp.async_control = async_control;
    if(async_control){
      SmartDashboard.putString("Sub Mode", "A");
    }else{
      SmartDashboard.putString("Sub Mode", "S");
    }
  }

  public boolean getAsync(){
    return async_control;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
