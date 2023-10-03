// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveMode;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Turret;

public class AdjustShooter extends CommandBase {
  private Turret m_turret;
  private DriveTrain m_train;
  private double powerY_last = 0.0;
  /** Creates a new AdjustShooter. */
  public AdjustShooter(Turret turret, DriveTrain train) {
    // Use addRequirements() here to declare subsystem dependencies.
    // addRequirements(turret);
    m_turret = turret;
    m_train = train;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(RobotContainer.m_driveMode==DriveMode.SWERVEDRIVE||RobotContainer.m_driveMode==DriveMode.ARCADEDRIVE){
      var Y = -RobotContainer.m_JoystickLeft.getY();
      var X = RobotContainer.m_JoystickLeft.getZ();
      var powerY = MathUtil.applyDeadband(Y, 0.4);
      var powerX = -MathUtil.applyDeadband(X, 0.4);
      var rightOutputX = MathUtil.applyDeadband(RobotContainer.m_JoystickRight.getY(), 0.02);
      var rightOutputY = MathUtil.applyDeadband(RobotContainer.m_JoystickRight.getX(), 0.02);
      var rightOutputZ = MathUtil.applyDeadband(RobotContainer.m_JoystickRight.getZ(), 0.2);
      if(powerY!=powerY_last){
        m_turret.adjustAngle(powerY);
        powerY_last = powerY;
      }
      if(rightOutputX==0&&rightOutputY==0&&rightOutputZ==0){
        m_train.drive(0, 0, powerX*2* Math.PI, false);
      }
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
