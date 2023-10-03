// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;

public class SwerveDrive extends CommandBase {

  private DriveTrain m_driveTrain;

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  public static boolean fieldRelative = false;

  /** Creates a new SwerveDrive. */
  public SwerveDrive(DriveTrain driveTrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driveTrain = driveTrain;
    SmartDashboard.putString("Sub Mode", "A");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(RobotContainer.m_driveMode==Constants.DriveMode.SWERVEDRIVE||(RobotContainer.m_driveMode==Constants.DriveMode.CLIMBUP&&!ClimbUp.async_control)){
      final var xSpeed =
          -m_xspeedLimiter.calculate(MathUtil.applyDeadband(RobotContainer.m_JoystickRight.getY(), 0.1))
              * DriveTrain.MaxVelocity;
      final var ySpeed =
          -m_yspeedLimiter.calculate(MathUtil.applyDeadband(RobotContainer.m_JoystickRight.getX(), 0.1))
              * DriveTrain.MaxVelocity;
      final var rot =
          -m_rotLimiter.calculate(MathUtil.applyDeadband(RobotContainer.m_JoystickRight.getZ(), 0.3))
              * Math.PI;
      if(RobotContainer.m_JoystickRight.getHID().getRawButtonPressed(1)){
        fieldRelative = false;
        SmartDashboard.putString("Sub Mode", "A");
      }else if(RobotContainer.m_JoystickRight.getHID().getRawButtonPressed(2)){
        fieldRelative = true;
        SmartDashboard.putString("Sub Mode", "F");
      }
      m_driveTrain.drive(xSpeed, ySpeed, rot, fieldRelative);
    }
    if(RobotContainer.m_driveMode==Constants.DriveMode.CLIMBUP&&ClimbUp.async_control){
      m_driveTrain.drive(0, 0, 0, fieldRelative);
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
