# FRC team 5449's code for 2022 _Rapid React_

Presented by our talented youth mentor, alumni, and team member from seanson 2020 to 2023 Zhang Jialin!

## [RobotContainer.java](https://github.com/FRC-team5449/prototype2022in23_final_version/blob/main/src/main/java/frc/robot/RobotContainer.java)

The class that plays a role of main class in the whole program, and all the commands are alligned with thier corresponding subsystems in this class. The joystick except swerve drive also maps with commands, which is achieved by Trigger.java, based on the control of Robot's state.

## [Autos.java](https://github.com/FRC-team5449/prototype2022in23_final_version/blob/main/src/main/java/frc/robot/commands/Autos.java)

The most stable Auto command we have created, which is attained with the help of the fabulous third party, (**PathPlanner**)[https://github.com/mjansen4857/pathplanner]. And the issue of runaway rotating Robot is solved by using different Path Constraints between each two waypoints.


