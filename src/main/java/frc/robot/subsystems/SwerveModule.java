// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveModule extends SubsystemBase {

  TalonFX m_driveMotor;
  TalonFX m_turningMotor;

  CANCoder m_turningEncoder;

  private final TrapezoidProfile.Constraints m_turningConstraints = new TrapezoidProfile.Constraints(360.0 * 150, 150 * 360.0);
  private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();

  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(Constants.driveKs, Constants.driveKv);
  private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(Constants.turnKs, Constants.turnKv);


  /** Creates a new SwerveModule. */
  public SwerveModule(int driveMotorChannel, int turningMotorChannel, int turningEncoderChannel) {
    m_driveMotor = new TalonFX(driveMotorChannel);
    m_turningMotor = new TalonFX(turningMotorChannel);

    m_turningEncoder = new CANCoder(turningEncoderChannel);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        m_driveMotor.getSelectedSensorVelocity()*Constants.ChassisWheelDistancePerPulse*10.0, new Rotation2d(Math.toRadians(m_turningEncoder.getPosition())));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        m_driveMotor.getSelectedSensorPosition()*Constants.ChassisWheelDistancePerPulse, new Rotation2d(Math.toRadians(m_turningEncoder.getPosition())));
  }

  private double last_cal = 0;
  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state =
        SwerveModuleState.optimize(desiredState, new Rotation2d(Math.toRadians(m_turningEncoder.getPosition())));
    var profile = new TrapezoidProfile(m_turningConstraints, new TrapezoidProfile.State(state.angle.getDegrees(),0), m_setpoint);
    m_setpoint = profile.calculate(Timer.getFPGATimestamp()-last_cal);
    last_cal = Timer.getFPGATimestamp();

    m_driveMotor.set(ControlMode.Velocity, state.speedMetersPerSecond/10.0/Constants.ChassisWheelDistancePerPulse, DemandType.ArbitraryFeedForward, m_driveFeedforward.calculate(state.speedMetersPerSecond));
    m_turningMotor.set(ControlMode.Position, m_setpoint.position, DemandType.ArbitraryFeedForward, m_turnFeedforward.calculate(m_setpoint.velocity));
  }

  public void setPower(double power){
    m_turningMotor.set(ControlMode.Position, 0.0);
    m_driveMotor.set(ControlMode.PercentOutput, power);
  }

  public double getPower(){
    return m_driveMotor.getMotorOutputPercent();
  }
}
