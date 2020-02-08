/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.EncoderType;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.elevatorConstants;
import frc.robot.RobotContainer;

public class elevatorSubsystem extends SubsystemBase {

  private Solenoid stage1Solenoid = new Solenoid(elevatorConstants.Solenoid1); // TODO: get correct ids
  private Solenoid stage2Solenoid = new Solenoid(elevatorConstants.Solenoid2);
  private Solenoid brakeSolenoid = new Solenoid(elevatorConstants.brakeSolenoidID);

  public final static CANSparkMax elevatorWinch = new CANSparkMax(elevatorConstants.elevatorWinch, MotorType.kBrushless);
  private final CANEncoder elevatorEncoder = elevatorWinch.getEncoder(EncoderType.kHallSensor, 2048);

  /**
   * Creates a new Climber.
   */
  public elevatorSubsystem() {

    stage1Solenoid.set(false);
    stage2Solenoid.set(false);

    elevatorWinch.restoreFactoryDefaults();
    elevatorWinch.setIdleMode(CANSparkMax.IdleMode.kBrake);
    elevatorEncoder.setInverted(false);

    RobotContainer.airCompressor.start();

  }

  public void deployStage1() {
    stage1Solenoid.set(true);
  }

  public void deployStage2() {
    stage2Solenoid.set(false);
  }

  public void retract1() {
    // TODO: lower climber so we can try again
  }

  public void retract2() {
  }

  public void raiseRobot() {
    brakeOff();
    elevatorWinch.setVoltage(6);
  }

  public void lowerRobot() {
    brakeOff();
    elevatorWinch.setVoltage(-2);
  }

  public void stopWinch() {
    elevatorWinch.setVoltage(0);
    brakeOn();
  }

  public void brakeOn() {
    brakeSolenoid.set(true);
  }

  public void brakeOff() {
    brakeSolenoid.set(false);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Winch_RPM", elevatorEncoder.getVelocity());
  }

}
