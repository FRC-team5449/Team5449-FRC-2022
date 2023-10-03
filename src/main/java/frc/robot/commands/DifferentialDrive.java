// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;

public class DifferentialDrive extends CommandBase {

  private DriveTrain m_driveTrain;
  /** Creates a new DifferentialDrive. */
  public DifferentialDrive(DriveTrain driveTrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driveTrain = driveTrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(RobotContainer.m_driveMode==Constants.DriveMode.TANKDRIVE){    // Tank Drive Mode
      var left = -RobotContainer.m_JoystickLeft.getY();
      var right = -RobotContainer.m_JoystickRight.getY();
      m_driveTrain.setDifferentialPower(left, right);
    }else if(RobotContainer.m_driveMode==Constants.DriveMode.ARCADEDRIVE){    // Arcade Drive Mode
      var forward = -RobotContainer.m_JoystickRight.getY();
      var traverse = RobotContainer.m_JoystickRight.getX();
      var rotational = MathUtil.applyDeadband(RobotContainer.m_JoystickRight.getZ(), 0.2)/2.0;
      var left = Math.min(Math.max(forward+rotational+Math.max(traverse, 0.0),-1.0),1);
      var right = Math.min(Math.max(forward-rotational-Math.min(traverse, 0.0),-1.0),1);
      m_driveTrain.setDifferentialPower(left, right);
    }
    // if(RobotContainer.m_JoystickRight.getHID().getRawButtonPressed(1)){
    //   m_driveTrain.gearUp();
    // }else if(RobotContainer.m_JoystickRight.getHID().getRawButtonReleased(1)){
    //   m_driveTrain.gearDown();
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
