/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;

public class colorSensor extends SubsystemBase {
  // space for variables
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  private final ColorMatch m_colorMatcher = new ColorMatch();
  String lastSeenColor = "Unknown";
  private int count = 0;
  /*
   * Color Wheel Blue CMY: 100,0,0 RGB: #00FFFF Green CMY: 100,0,100 RGB: #00FF00 Red CMY: 0,100,100
   * RGB: #FF0000 Yellow CMY: 0,0,100 RGB: #FFFF00
   */
  private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
  private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
  private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
  private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);

  public colorSensor() {
    // colors we want to match
    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kGreenTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
    m_colorMatcher.addColorMatch(kYellowTarget);
  }

  public void senseColorWheelPos() {
    // This method will be called once per scheduler run
    Color detectedColor = m_colorSensor.getColor();


    /**
     * Run the color match algorithm on our detected color
     */
    String colorString;
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

    if (match.color == kBlueTarget) {
      colorString = "Blue";
    } else if (match.color == kRedTarget) {
      colorString = "Red";
    } else if (match.color == kGreenTarget) {
      colorString = "Green";
    } else if (match.color == kYellowTarget) {
      colorString = "Yellow";
    } else {
      colorString = "Unknown";
    }

    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("Confidence", match.confidence);
    SmartDashboard.putString("Detected Color", colorString);

    int proximity = m_colorSensor.getProximity();
    SmartDashboard.putNumber("Proximity", proximity);

    /* Code for counting colors */

    if (lastSeenColor.equals("Red")) {
      if (colorString.equals("Green")) {
        count = count + 1;
      }
      if (colorString.equals("Yellow")) {
        count = count - 1;
      }
    } else if (lastSeenColor.equals("Green")) {
      if (colorString.equals("Blue")) {
        count = count + 1;
      }
      if (colorString.equals("Red")) {
        count = count - 1;
      }
    } else if (lastSeenColor.equals("Blue")) {
      if (colorString.equals("Yellow")) {
        count = count + 1;
      }
      if (colorString.equals("Green")) {
        count = count - 1;
      }
    } else if (lastSeenColor.equals("Yellow")) {
      if (colorString.equals("Red")) {
        count = count + 1;
      }
      if (colorString.equals("Blue")) {
        count = count - 1;
      }
    }
    // Color reset and count display on SmartDashboard
    lastSeenColor = colorString;
    SmartDashboard.putNumber("Count", count);

    // TODO: Detect errors and unknown colors

    // System.out.println("Color Change Count: " + count);



    /*
     * Print out for colorString
     * 
     * if (colorString != lastSeenColor) { System.out.println("Color: " + colorString);
     * lastSeenColor = colorString; }
     */
  }

  public double getRotationCount() {
    return (count / 8);
  }
}

