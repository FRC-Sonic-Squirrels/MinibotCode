/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

public class DriveSubsystem extends SubsystemBase {
  private final DifferentialDrive drive;
  private final RelativeEncoder m_leftEncoder;
  private final RelativeEncoder m_rightEncoder;

  private final double GEARRATIO = 0.0556;


  public DriveSubsystem() {
    /**
     * Class variables for storing data and objects that this subsystem needs to
     * operate.
     *
     * These variables are declared private to this class, because no other object
     * should need direct access to these objects. Any changes or commands to the
     * drivetrain should come through public methods like the tankDrive() function.
     */
    CANSparkMax leftNEO = new CANSparkMax(Constants.DriveConstants.LEFT_NEO_CANID, MotorType.kBrushless);
    CANSparkMax rightNEO = new CANSparkMax(Constants.DriveConstants.RIGHT_NEO_CANID, MotorType.kBrushless);
   
    // set all NEOs to factory defaults
    leftNEO.restoreFactoryDefaults();
    rightNEO.restoreFactoryDefaults();

    m_leftEncoder = leftNEO.getEncoder();
    m_rightEncoder = rightNEO.getEncoder();

    // assign each motor to a MotorControllerGroup
    MotorController leftSide = new MotorControllerGroup(leftNEO);
    MotorController rightSide = new MotorControllerGroup(rightNEO);

    // create our DifferentialDrive class
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

  public double getDistanceInches(){
    double REVOLUTIONS_TO_INCHES = GEARRATIO * (6 * Math.PI);
    double leftValue = m_leftEncoder.getPosition() * REVOLUTIONS_TO_INCHES;
    double rightValue = m_rightEncoder.getPosition() * REVOLUTIONS_TO_INCHES;
    return (leftValue + rightValue) / 2; // Return average value
  }
}
