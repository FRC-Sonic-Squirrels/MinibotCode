/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
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
  public blinkin(int pwmPort) {
    m_blinkin = new Spark(pwmPort);
    solid_orange();
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
  public void set(double val) {
    if ((val >= -1.0) && (val <= 1.0)) {
      m_blinkin.set(val);
    }
  }

  public void rainbow() {
    set(-0.99);
  }

  public void solid_orange() {
    set(0.63);
  }

  /* 
  * Add listeners to trigger on changes to the NetworkTable
  *
  * NetworkTable key for Alliance color:
  *    FMSInfo -> IsRedAlliance -> value: boolean (True|False)
  *
  * NetworkTable for Limelight in valid target
  *    limelight -> tv -> value: 0 or 1
  *
  */
  public void allianceColor() {
    NetworkTable t = NetworkTableInstance.getDefault().getTable("FMSInfo");  
    t.addEntryListener("IsRedAlliance", (table, key, entry, value, flags) -> {  
      if (value.getBoolean() == true){
        RobotContainer.m_blinkin.set(-0.01);
        System.out.println("led RED");
      } else {
        RobotContainer.m_blinkin.set(0.19);
        System.out.println("led BLUE");
      }
    }, TableEntryListener.kNew | TableEntryListener.kUpdate | TableEntryListener.kLocal);
    
  }
  

  /**
  private void allianceColor() {
    final NetworkTableEntry tableAllianceColor = NetworkTableInstance.getDefault().getTable("FMSInfo").getEntry("IsRedAlliance");
    tableAllianceColor.addListener(event -> {
      System.out.println(event.value);
        if (event.value.getBoolean() == true){
          RobotContainer.m_blinkin.set(-0.01);
          System.out.println("led RED"); 
        } else {
          RobotContainer.m_blinkin.set(0.59);
          System.out.println("led BLUE");
        }
    }, TableEntryListener.kNew | TableEntryListener.kUpdate);
  }
  */
  
}
