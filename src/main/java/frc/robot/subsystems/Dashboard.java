// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveMode;
import frc.robot.commands.ClimbUp;
import frc.robot.commands.SwerveDrive;

public class Dashboard extends SubsystemBase {
  DriveTrain source1;
  Compressor source2;
  Turret source3;
  LimeLight source4;
  DBSendable m_dbSendable;
  DriveMode m_driveMode_current;
  NetworkTable dashboard_NT;
  DoubleArraySubscriber selfposreader;
  double last_timestamp = 0;
  /** Creates a new Dashboard. */
  public Dashboard(DriveTrain source1, Compressor source2, Turret source3, LimeLight source4) {
    this.source1 = source1;
    String[] modes = new String[Constants.AutoMode.values().length];
    for(int i=0;i<Constants.AutoMode.values().length;i++){
      modes[i]=Constants.AutoMode.values()[i].name();
    }
    SmartDashboard.putStringArray("Auto List", modes);
    SmartDashboard.putString("Auto Selector", RobotContainer.m_autoMode.name());
    this.source2 = source2;
    m_driveMode_current = null;
    this.source3 = source3;
    dashboard_NT = NetworkTableInstance.getDefault().getTable("SmartDashboard");
    selfposreader = dashboard_NT.getDoubleArrayTopic("Set Position").subscribe(new double[]{source1.getPose2d().getX(),source1.getPose2d().getY(),source1.getHeading()});
    selfposreader.readQueueValues();
    this.source4 = source4;
    m_dbSendable = new DBSendable(source4, source3);
    SmartDashboard.putData("DB",m_dbSendable);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    try{
      var new_automode = Constants.AutoMode.valueOf(SmartDashboard.getString("Auto Selector", RobotContainer.m_autoMode.name()));
      if(RobotContainer.m_autoMode != new_automode){
        RobotContainer.m_autoMode = new_automode;
        // Start Points
        switch(RobotContainer.m_autoMode){
          case PATH_BOTTOM:
            source1.setPose2d(PathPlanner.loadPath("PATH_BOTTOM", new PathConstraints(4, 3)).getInitialHolonomicPose());
            break;
          case PATH_BOTTOM_MIDDLE_HOMING:
            source1.setPose2d(PathPlanner.loadPath("PATH_BOTTOM_MIDDLE_HOMING", new PathConstraints(4, 3)).getInitialHolonomicPose());
            break;
          default:
            break;
        }
      }
    }catch(Exception e){}
    // System.out.println("AutoMode: "+RobotContainer.m_autoMode);
    try{
      for(double[] coords:selfposreader.readQueueValues()){
        var new_pos = new Pose2d(coords[0], coords[1], Rotation2d.fromDegrees(coords[2]));
        source1.setPose2d(new_pos);
      }
    }catch(Exception e){}
    SmartDashboard.putNumberArray("RobotDrive Motors", new double[]{source1.getPower()[0],source1.getPower()[1],source1.getPower()[2],source1.getPower()[3]});
    SmartDashboard.putNumber("Gyro", 360-MathUtil.inputModulus(source1.getHeading(), 0.0, 360.0));
    if(Timer.getFPGATimestamp()-last_timestamp>0.2){
      SmartDashboard.putNumber("Air Pressure", source2.getPressure());
      last_timestamp = Timer.getFPGATimestamp();
    }
    if(m_driveMode_current!=RobotContainer.m_driveMode){
      SmartDashboard.putNumber("DriveMode", RobotContainer.m_driveMode.ordinal());
      if(RobotContainer.m_driveMode==DriveMode.SWERVEDRIVE){
        SmartDashboard.putString("Sub Mode", (SwerveDrive.fieldRelative?"F":"A"));
      }else if(RobotContainer.m_driveMode==DriveMode.CLIMBUP){
        SmartDashboard.putString("Sub Mode", (ClimbUp.async_control?"A":"S"));
      }
      m_driveMode_current = RobotContainer.m_driveMode;
    }
    SmartDashboard.putNumberArray("Self Position", new double[]{source1.getPose2d().getX(),source1.getPose2d().getY(),source1.getHeading()});
    SmartDashboard.putNumber("Emission Speed", Transit.ShooterSpeed/Constants.ShooterMaxSpeed*Constants.EmissionMaxSpeed);
    SmartDashboard.putNumber("Turret Angle", source3.getAngle());
    SmartDashboard.putNumber("Shoot Distance", source1.getPose2d().getTranslation().getDistance(new Translation2d(8.2296, 4.1148)));
    SmartDashboard.putNumber("Balls", Transit.ballsShooted);
  }
}
