// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {

  public static double MaxVelocity = Constants.kMaxSpeed;

  // Differential Wheel Modules  !!! Deconstructed - 2023/4/11
  // private DifferentialModel leftModel = new DifferentialModel(DifferentialModelType.LEFTMODEL);
  // private DifferentialModel rightModel = new DifferentialModel(DifferentialModelType.RIGHTMODEL);
  // private static Solenoid gear = new Solenoid(Constants.PneumaticHub, PneumaticsModuleType.CTREPCM, 5);

  // Swerve Drive Modules
  private final Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
  private final Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
  private final Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
  private final Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);
  
  private SwerveModule m_frontLeft = new SwerveModule(Constants.LeftFMotorID, Constants.LeftFServoID, Constants.LeftFEncoderID);
  private SwerveModule m_frontRight = new SwerveModule(Constants.RightFMotorID, Constants.RightFServoID, Constants.RightFEncoderID);
  private SwerveModule m_backLeft = new SwerveModule(Constants.LeftBMotorID, Constants.LeftBServoID, Constants.LeftBEncoderID);
  private SwerveModule m_backRight = new SwerveModule(Constants.RightBMotorID, Constants.RightBServoID, Constants.RightBEncoderID);

  public AHRS m_gyro = new AHRS();
  // Kinematics
  public final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
          m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  private final SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(
          m_kinematics,
          m_gyro.getRotation2d(),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
          });

  /** Creates a new DriveTrain. */
  public DriveTrain() {
    m_gyro.reset();
  }

  // public void setDifferentialPower(double leftPower, double rightPower){
  //   leftModel.setPower(leftPower);
  //   rightModel.setPower(rightPower);
  // }

  public void setDifferentialPower(double leftPower, double rightPower){
    m_frontLeft.setPower(leftPower);
    m_backLeft.setPower(leftPower);
    m_frontRight.setPower(rightPower);
    m_backRight.setPower(rightPower);
  }

  // public double[] getPower(){
  //   return new double[]{leftModel.getPower(),rightModel.getPower(),leftModel.getPower(),rightModel.getPower()};
  // }

  public double[] getPower(){
    return new double[]{m_frontLeft.getPower(),m_frontRight.getPower(),m_backLeft.getPower(),m_backRight.getPower()};
  }

  public double getHeading(){
    return MathUtil.inputModulus(m_gyro.getRotation2d().getDegrees(), -180.0, 180.0);
  }

  public void resetgyro(){
    m_gyro.reset();
  }

  public void resetgyro(double deg){
    m_gyro.zeroYaw();
    m_gyro.setAngleAdjustment(-deg);
  }

  public Pose2d getPose2d(){
    return m_odometry.getPoseMeters();
  }

  public void setPose2d(Pose2d position){
    var rotation = MathUtil.inputModulus(position.getRotation().getDegrees(), -180.0, 180.0);
    resetgyro(rotation);
    m_odometry.resetPosition(m_gyro.getRotation2d(), new SwerveModulePosition[] {
      m_frontLeft.getPosition(),
      m_frontRight.getPosition(),
      m_backLeft.getPosition(),
      m_backRight.getPosition()
    }, position);
  }

  // public void gearUp(){
  //   gear.set(true);
  // }

  // public void gearDown(){
  //   gear.set(false);
  // }


  private double xSpeed, ySpeed;
  private boolean fieldRelative;
   /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, MaxVelocity);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
    this.xSpeed = xSpeed;
    this.ySpeed = ySpeed;
    this.fieldRelative = fieldRelative;
  }

  public void drive(double rot){
    drive(xSpeed, ySpeed, rot, fieldRelative);
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, MaxVelocity);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_backLeft.setDesiredState(desiredStates[2]);
    m_backRight.setDesiredState(desiredStates[3]);
  }

  public void updateOdometry() {
    m_odometry.update(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()
        });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateOdometry();
  }
}
