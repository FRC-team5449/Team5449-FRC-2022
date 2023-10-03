// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.Constants;
import frc.robot.RobotContainer;

/** Add your docs here. */
public class DBSendable implements Sendable {
    private boolean MaxVelocityLocked = true;
    private double rightThrottleHis = 0.0;
    private boolean ShooterSpeedLocked = true;
    private double leftThrottleHis = 0.0;
    private LimeLight source1;
    private Turret source2;
    public DBSendable(LimeLight source1, Turret source2){
        this.source1 = source1;
        this.source2 = source2;
    }
    public double MaxVelocityGetter(){
        var val = (-RobotContainer.m_JoystickRight.getThrottle()+1.0)/2.0*Constants.kMaxSpeed;
        if(MaxVelocityLocked&&Math.abs(RobotContainer.m_JoystickRight.getThrottle()-rightThrottleHis)<0.2){
            return DriveTrain.MaxVelocity;
        }
        MaxVelocityLocked = false;
        DriveTrain.MaxVelocity = val;
        return val;
    }
    public void MaxVelocitySetter(double val){
        MaxVelocityLocked = true;
        rightThrottleHis = RobotContainer.m_JoystickRight.getThrottle();
        DriveTrain.MaxVelocity = val;
    }
    public double ShooterSpeedGetter(){
        var val = (-RobotContainer.m_JoystickLeft.getThrottle()+1.0)/2.0*Constants.ShooterMaxSpeed;
        if(ShooterSpeedLocked&&Math.abs(RobotContainer.m_JoystickLeft.getThrottle()-leftThrottleHis)<0.2){
            return Transit.ShooterSpeed/Constants.ShooterMaxSpeed;
        }
        ShooterSpeedLocked = false;
        Transit.ShooterSpeed = val;
        return val/Constants.ShooterMaxSpeed;
    }
    public void ShooterSpeedSetter(double val){
        ShooterSpeedLocked = true;
        leftThrottleHis = RobotContainer.m_JoystickLeft.getThrottle();
        Transit.ShooterSpeed = val*Constants.ShooterMaxSpeed;
    }
    public void DisableLightingSetter(boolean val){
        if(val)
            source1.turnOffLed();
        else
            source1.turnOnLed();
    }
    public void TurretAngleSetter(double val){
        source2.setAngle(val);
    }
    public double TurretAngleGetter(){
        return source2.getAngle();
    }
    public void initSendable(SendableBuilder builder){
        // builder.setSmartDashboardType("DB");
        builder.addDoubleProperty("Slider 0", this::MaxVelocityGetter, this::MaxVelocitySetter);
        builder.addDoubleProperty("Slider 1", this::ShooterSpeedGetter, this::ShooterSpeedSetter);
        builder.addDoubleProperty("Slider 2", this::TurretAngleGetter, this::TurretAngleSetter);
        builder.addBooleanProperty("Button 3", null, this::DisableLightingSetter);
    }
}
