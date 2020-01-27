/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TableEntryListener;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;


public class blinkin extends SubsystemBase {

  /*
   * Rev Robotics Blinkin takes a PWM signal from 1000-2000us This is identical to
   * a SparkMax motor. -1 corresponds to 1000us 0 corresponds to 1500us +1
   * corresponds to 2000us
   */
  private static Spark m_blinkin = null;

  /**
   * Creates a new Blinkin LED controller.
   * 
   * @param pwmPort The PWM port the Blinkin is connected to.
   */
  public blinkin(final int pwmPort) {
    m_blinkin = new Spark(pwmPort);
    solidOrange();
    limelightListener();
    allianceColorListener();
  }

  /*
   * Set the color and blink pattern of the LED strip.
   * 
   * Consult the Rev Robotics Blinkin manual Table 5 for a mapping of values to
   * patterns.
   * 
   * @param val The LED blink color and patern value [-1,1]
   * 
   */
  public void set(final double val) {
    if ((val >= -1.0) && (val <= 1.0)) {
      m_blinkin.set(val);
    }
  }

  public void solidBlue() {
    set(0.87);
  }

  public void solidRed() {
    set(0.61);
  }

  public void flashingBlue() {
    set(-0.23);
  }

  public void strobeBlue() {
    set(-0.09);
  }

  public void solidOrange() {
    set(0.63);
  }

  /*
   * Add listeners to trigger on changes to the NetworkTable
   *
   * NetworkTable key for Alliance color: FMSInfo -> IsRedAlliance -> value:
   * (True|False)
   *
   * NetworkTable for Limelight in valid target limelight -> tv -> value: (0 | 1)
   * // On valid target limelight -> ta -> value: (% of image on target)
   *
   */

  private void limelightListener() {
    NetworkTable tableLimelight = NetworkTableInstance.getDefault().getTable("limelight-one");
    
    /* Add listeners */
    tableLimelight.addEntryListener("tx", (table, key, entry, value, flags) -> {
      double targetX = value.getDouble();

      if (targetX >= -1.0 && targetX <= 1.0) {
          solidBlue();
          System.out.println("LED Solid Blue");
      } else if ((targetX < -1.0  && targetX > -29.8) || (targetX > 1.0  && targetX < 29.8)) {
          flashingBlue();
          System.out.println("LED Flashing Blue");
      } else {
          solidOrange();
          System.out.println("LED Solid Orange");
      }
      }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
  }  

    
  private void allianceColorListener() {
    final NetworkTableEntry tableAllianceColor = NetworkTableInstance.getDefault().getTable("FMSInfo")
        .getEntry("IsRedAlliance");
      tableAllianceColor.addListener(event -> {
        if (event.value.getBoolean() == true){
          solidRed();
          System.out.println("led RED"); 
        } else {
          solidBlue();
          System.out.println("led BLUE");
        }
    }, TableEntryListener.kNew | TableEntryListener.kUpdate | TableEntryListener.kLocal) ;
  }

public void blinkin() {
}

}
