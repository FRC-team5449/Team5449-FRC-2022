// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ColorSensor extends SubsystemBase {
  private final ColorSensorV3 m_colorSensorLower = new ColorSensorV3(I2C.Port.kOnboard);
  private final ColorSensorV3 m_colorSensorUpper = new ColorSensorV3(I2C.Port.kMXP);
  private final Color kBlueTarget = new Color(0.143, 0.100, 0.429);
  private final Color kRedTarget = new Color(0.561, 0.232, 0.114);
  private final Color kEmpty = new Color(0.150, 0.150, 0.150); 
  private final ColorMatch m_colorMatcher = new ColorMatch();
  private Constants.BallNiche nicheLower;
  private Constants.BallNiche nicheUpper;
  /** Creates a new ColorSensor. */
  public ColorSensor() {
    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
    m_colorMatcher.addColorMatch(kEmpty);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Color detectedColorLower = m_colorSensorLower.getColor();
    Color detectedColorUpper = m_colorSensorUpper.getColor();
    /**
     * Run the color match algorithm on our detected color
     */
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColorLower);
    if (match.color == kBlueTarget) {
      nicheLower = Constants.BallNiche.BLUE;
    } else if (match.color == kRedTarget) {
      nicheLower = Constants.BallNiche.RED;
    } else {
      nicheLower = Constants.BallNiche.EMPTY;
    }
    
    ColorMatchResult match2 = m_colorMatcher.matchClosestColor(detectedColorUpper);
    if (match2.color == kBlueTarget) {
      nicheUpper = Constants.BallNiche.BLUE;
    } else if (match2.color == kRedTarget) {
      nicheUpper = Constants.BallNiche.RED;
    } else {
      nicheUpper = Constants.BallNiche.EMPTY;
    }

    SmartDashboard.putNumber("BallNicheLower", nicheLower.ordinal());
    SmartDashboard.putNumber("BallNicheUpper", nicheUpper.ordinal());
  }

  public Constants.BallNiche[] getNiches(){
    return new Constants.BallNiche[]{nicheLower, nicheUpper};
  }
}
