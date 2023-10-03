// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.AutoMode;
import frc.robot.Constants.DriveMode;
import frc.robot.commands.AdjustShooter;
import frc.robot.commands.AlignTurretCmd;
import frc.robot.commands.Autos;
import frc.robot.commands.BallScreeningCmd;
import frc.robot.commands.ClimbUp;
import frc.robot.commands.ClimbUpCmd;
import frc.robot.commands.Detrude;
import frc.robot.commands.DetrudeOnce;
import frc.robot.commands.DifferentialDrive;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.Shoot;
import frc.robot.commands.SpoutOnce;
import frc.robot.commands.SwerveDrive;
import frc.robot.commands.UpLift;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.Dashboard;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.Transit;
import frc.robot.subsystems.Turret;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // Robot State Params
  public static Constants.DriveMode m_driveMode = Constants.DriveMode.SWERVEDRIVE;
  public static Constants.AutoMode m_autoMode = Constants.AutoMode.AP;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  public static final CommandJoystick m_JoystickLeft = new CommandJoystick(1);
  public static final CommandJoystick m_JoystickRight = new CommandJoystick(0);

  // Pneumatics
  private final Compressor phCompressor = new Compressor(Constants.PneumaticHub, PneumaticsModuleType.REVPH);

  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final DriveTrain m_driveTrain = new DriveTrain();
  private final Transit m_transit = new Transit();
  private final Turret m_turret = new Turret();
  private final Climber m_climber = new Climber();
  private final ColorSensor m_colorSensor = new ColorSensor();
  private final LimeLight m_limeLight = new LimeLight();
  private final Dashboard m_dashboard = new Dashboard(m_driveTrain, phCompressor, m_turret, m_limeLight);

  public final DifferentialDrive m_differentialDrive = new DifferentialDrive(m_driveTrain);
  public final SwerveDrive m_swerveDrive = new SwerveDrive(m_driveTrain);
  public final Shoot m_shoot = new Shoot(m_transit);
  public final SpoutOnce m_spoutOnce = new SpoutOnce(m_transit);
  public final UpLift m_upLift = new UpLift(m_transit);
  public final Detrude m_detrude = new Detrude(m_transit);
  public final DetrudeOnce m_detrudeOnce = new DetrudeOnce(m_transit, m_colorSensor);
  public final AdjustShooter m_adjustShooter = new AdjustShooter(m_turret, m_driveTrain);
  public final ClimbUpCmd m_climbUpCmd = new ClimbUpCmd(m_climber, m_driveTrain.m_gyro);
  public final ClimbUp m_climbUp = new ClimbUp(m_climber, m_climbUpCmd);
  public final AlignTurretCmd m_alignTurretCmd = new AlignTurretCmd(m_driveTrain, m_limeLight);
  public final BallScreeningCmd m_ballScreeningCmd = new BallScreeningCmd(m_colorSensor, m_transit);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    m_driveTrain.resetgyro();
    m_driveTrain.setPose2d(new Pose2d(0, 0, new Rotation2d(0)));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule all commands to the joysticks
    m_JoystickLeft.button(1).and(()->RobotContainer.m_driveMode!=Constants.DriveMode.CLIMBUP).whileTrue(m_shoot);
    m_JoystickLeft.button(5).whileTrue(m_upLift);
    m_JoystickLeft.button(3).whileTrue(m_detrude);
    m_JoystickLeft.button(6).onTrue(Commands.runOnce(()->m_transit.armDown()));
    m_JoystickLeft.button(4).onTrue(Commands.runOnce(()->m_transit.armUp()));
    m_JoystickLeft.button(2).and(()->RobotContainer.m_driveMode!=Constants.DriveMode.CLIMBUP).onTrue(Commands.runOnce(()->m_transit.armDown())).whileTrue(m_upLift).onFalse(Commands.runOnce(()->m_transit.armUp()));
    m_JoystickLeft.povUp().and(()->RobotContainer.m_driveMode==Constants.DriveMode.TANKDRIVE).whileTrue(Commands.startEnd(()->m_turret.adjustAngle(0.5), ()->m_turret.adjustAngle(0)));
    m_JoystickLeft.povDown().and(()->RobotContainer.m_driveMode==Constants.DriveMode.TANKDRIVE).whileTrue(Commands.startEnd(()->m_turret.adjustAngle(-0.3), ()->m_turret.adjustAngle(0)));
    m_JoystickLeft.povUp().or(m_JoystickLeft.povUpLeft()).or(m_JoystickLeft.povUpRight()).and(()->RobotContainer.m_driveMode!=Constants.DriveMode.TANKDRIVE).and(()->RobotContainer.m_driveMode!=Constants.DriveMode.CLIMBUP).whileTrue(m_spoutOnce);
    m_JoystickLeft.povDown().or(m_JoystickLeft.povDownLeft()).or(m_JoystickLeft.povDownRight()).and(()->RobotContainer.m_driveMode!=Constants.DriveMode.TANKDRIVE).and(()->RobotContainer.m_driveMode!=Constants.DriveMode.CLIMBUP).whileTrue(m_detrudeOnce);
    m_JoystickLeft.povRight().or(m_JoystickRight.povRight()).and(()->RobotContainer.m_driveMode==Constants.DriveMode.CLIMBUP).onTrue(Commands.runOnce(()->m_climbUp.setAsync(true)));
    m_JoystickLeft.povLeft().or(m_JoystickRight.povLeft()).and(()->RobotContainer.m_driveMode==Constants.DriveMode.CLIMBUP).onTrue(Commands.runOnce(()->m_climbUp.setAsync(false)));
    m_JoystickLeft.button(1).and(()->RobotContainer.m_driveMode==Constants.DriveMode.CLIMBUP).onTrue(m_climbUpCmd);
    m_JoystickLeft.button(7).or(m_JoystickRight.button(7)).onTrue(Commands.runOnce(()->m_driveMode=DriveMode.AP, m_dashboard));
    m_JoystickLeft.button(8).or(m_JoystickRight.button(8)).onTrue(Commands.runOnce(()->m_driveMode=DriveMode.SWERVEDRIVE, m_dashboard));
    m_JoystickLeft.button(9).or(m_JoystickRight.button(9)).onTrue(Commands.runOnce(()->m_driveMode=DriveMode.ARCADEDRIVE, m_dashboard));
    m_JoystickLeft.button(10).or(m_JoystickRight.button(10)).onTrue(Commands.runOnce(()->m_driveMode=DriveMode.TANKDRIVE, m_dashboard));
    m_JoystickLeft.button(11).or(m_JoystickRight.button(11)).onTrue(Commands.runOnce(()->m_driveMode=DriveMode.CLIMBUP, m_dashboard));
    m_JoystickLeft.button(12).or(m_JoystickRight.button(12)).whileTrue(m_alignTurretCmd);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    if(m_autoMode!=AutoMode.AP){
      return Autos.follow_trajectory(m_autoMode, m_driveTrain, m_transit, m_colorSensor, m_limeLight, m_upLift, m_shoot, m_detrude);
    }else{
      return Autos.exampleAuto(m_exampleSubsystem);
    }
  }
}
