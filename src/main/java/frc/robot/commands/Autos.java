// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Constants.BallNiche;
import frc.robot.subsystems.*;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;

public final class Autos {

    /** Example static factory for an autonomous command. */
    public static CommandBase exampleAuto(ExampleSubsystem subsystem) {
        return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
    }

    public static CommandBase follow_trajectory(Constants.AutoMode nPath,
            DriveTrain driveTrain,
            Transit transit,
            ColorSensor colorSensor,
            LimeLight limeLight,
            UpLift upliftcmd,
            Shoot shoot,
            Detrude detrudecmd) {
        List<PathPlannerTrajectory> pathGroup;
        switch (nPath) {
            case AP:
                return Commands.none();
            case PATH_BOTTOM:
                SmartDashboard.putNumber("DB/Slider 1", 0.7);
                SmartDashboard.putNumber("DB/Slider 2", 60.0);
                pathGroup = PathPlanner.loadPathGroup("PATH_BOTTOM",
                        new PathConstraints(4.5, 2.5),
                        new PathConstraints(0.5, 0.25));
                break;
            case PATH_BOTTOM_MIDDLE:
                SmartDashboard.putNumber("DB/Slider 1", 0.7);
                SmartDashboard.putNumber("DB/Slider 2", 60.0);
                pathGroup = PathPlanner.loadPathGroup("PATH_BOTTOM_MIDDLE",
                        new PathConstraints(5, 3),
                        new PathConstraints(0.75, 0.5),
                        new PathConstraints(5, 3),
                        new PathConstraints(1.5, 1));
                break;
            case PATH_TOP:
                SmartDashboard.putNumber("DB/Slider 1", 0.7);
                SmartDashboard.putNumber("DB/Slider 2", 60.0);
                pathGroup = PathPlanner.loadPathGroup("PATH_TOP",
                        new PathConstraints(7.5, 3.5),
                        new PathConstraints(1.5, 1),
                        new PathConstraints(6.5, 3),
                        new PathConstraints(5, 2.5),
                        new PathConstraints(2, 1.5));
                break;
            case PATH_MIDDLE:
                SmartDashboard.putNumber("DB/Slider 1", 0.7);
                SmartDashboard.putNumber("DB/Slider 2", 60.0);
                pathGroup = PathPlanner.loadPathGroup("PATH_MIDDLE",
                        new PathConstraints(4.5, 2.5),
                        new PathConstraints(2, 1.5));
                break;
            case PATH_BOTTOM_MIDDLE_HOMING:
                SmartDashboard.putNumber("DB/Slider 1", 0.7);
                SmartDashboard.putNumber("DB/Slider 2", 60.0);
                pathGroup = PathPlanner.loadPathGroup("PATH_BOTTOM_MIDDLE_HOMING_ORGINAL",
                        new PathConstraints(4, 3),
                        new PathConstraints(2, 1),
                        new PathConstraints(4, 3),
                        new PathConstraints(4, 3),
                        new PathConstraints(4, 3),
                        new PathConstraints(4, 3));
                break;
            case PATH_MIDDLE_BOTTOM_HOMING:
                SmartDashboard.putNumber("DB/Slider 1", 0.7);
                SmartDashboard.putNumber("DB/Slider 2", 60.0);
                pathGroup = PathPlanner.loadPathGroup("PATH_MIDDLE_BOTTOM_HOMING",
                        new PathConstraints(5, 3),
                        new PathConstraints(1, 0.75),
                        new PathConstraints(5, 3),
                        new PathConstraints(5, 3),
                        new PathConstraints(1, 0.75),
                        new PathConstraints(1, 0.75),
                        new PathConstraints(1, 0.75));
                break;
            case PATH_BOTTOM_TOP_HOMING:
                SmartDashboard.putNumber("DB/Slider 1", 0.7);
                SmartDashboard.putNumber("DB/Slider 2", 60.0);
                pathGroup = PathPlanner.loadPathGroup("PATH_BOTTOM_TOP_HOMING",
                        new PathConstraints(4, 3),
                        // new PathConstraints(2, 1),
                        new PathConstraints(4, 3),
                        new PathConstraints(4, 3),
                        new PathConstraints(4, 3),
                        new PathConstraints(4, 3));
                break;
            default:
                return Commands.none();
        }
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("startShoot", Commands.runOnce(()->shoot.schedule()));
        eventMap.put("shoot", Commands.deadline(new ShootCmd(transit, colorSensor),new AlignTurretCmd(driveTrain, limeLight)));
        eventMap.put("stopShoot", Commands.runOnce(()->shoot.cancel()));
        eventMap.put("startUpLift", Commands.runOnce(() -> upliftcmd.schedule()));
        eventMap.put("upLiftCmd", Commands.race(Commands.waitSeconds(2), new FunctionalCommand(() -> upliftcmd.schedule(), () -> {}, interrupted->upliftcmd.cancel(), ()->colorSensor.getNiches()[0]!=BallNiche.EMPTY&&colorSensor.getNiches()[1]!=BallNiche.EMPTY)));
        eventMap.put("stopUpLift", Commands.runOnce(() -> upliftcmd.cancel()));
        eventMap.put("armDown", Commands.runOnce(() -> transit.armDown()));
        eventMap.put("armUp", Commands.runOnce(() -> transit.armUp()));
        eventMap.put("startDetrude", Commands.runOnce(() -> detrudecmd.schedule()));
        eventMap.put("stopDetrude", Commands.runOnce(() -> detrudecmd.cancel()));
        eventMap.put("align", new AlignTurretCmd(driveTrain, limeLight));
        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
                driveTrain::getPose2d, // Pose2d supplier
                driveTrain::setPose2d, // Pose2d consumer, used to reset odometry at the beginning of auto
                driveTrain.m_kinematics, // SwerveDriveKinematics
                new PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X
                                                 // and Y PID controllers)
                new PIDConstants(2.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the
                                                 // rotation controller)
                driveTrain::setModuleStates, // Module states consumer used to output to the drive subsystem
                eventMap,
                false, // Should the path be automatically mirrored depending on alliance color.
                       // Optional, defaults to true
                driveTrain // The drive subsystem. Used to properly set the requirements of path following
                           // commands
        );
        return autoBuilder.fullAuto(pathGroup);
    }

    private Autos() {
        throw new UnsupportedOperationException("This is a utility class!");
    }
}
