/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

public class DriveSubsystem extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */
  public static final CANSparkMax neo1 = new CANSparkMax(Constants.DriveConstants.NEO_1, MotorType.kBrushless);
  public static final CANSparkMax neo2 = new CANSparkMax(Constants.DriveConstants.NEO_2, MotorType.kBrushless);
  public static MotorController leftSide;
  public static MotorController rightSide;
  public static DifferentialDrive drive;

  public DriveSubsystem() {
    // set all NEOs to factory defaults
    neo1.restoreFactoryDefaults();
    neo2.restoreFactoryDefaults();
    leftSide = new MotorControllerGroup(neo1);
    rightSide = new MotorControllerGroup(neo2);
    drive = new DifferentialDrive(leftSide, rightSide);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void arcadeDrive(double xSpeed, double zRotation) {
    drive.arcadeDrive(xSpeed, zRotation);
  }

  public void tankDrive(double leftSpeed, double rightSpeed) {
    drive.tankDrive(leftSpeed, rightSpeed);
  }
}
