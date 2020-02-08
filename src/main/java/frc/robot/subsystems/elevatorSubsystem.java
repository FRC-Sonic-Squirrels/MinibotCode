/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.elevatorConstants;
import frc.robot.commands.elevatorWinch;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;

public class elevatorSubsystem extends SubsystemBase {

  private double P;
  private double I;
  private double D;
  private double F;



  private Solenoid leftSolenoid1=new Solenoid(1);
  private Solenoid leftSolenoid2=new Solenoid(1, 1);

  private Solenoid rightSolenoid1=new Solenoid(1);
  private Solenoid rightSolenoid2=new Solenoid(1, 1);

  public final static WPI_TalonFX elevatorWinch = new WPI_TalonFX(elevatorConstants.elevatorWinch);

  public class airsystem extends Pneumatics {

  
    
  }

  /**
   * Creates a new Climber.
   */
  public elevatorSubsystem() {
    elevatorWinch.configFactoryDefault();
    elevatorWinch.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor,
        elevatorConstants.elevatorSlotIdx,
        elevatorConstants.elevatorPivotTimeout);
    elevatorWinch.setSensorPhase(true);
    elevatorWinch.configNominalOutputForward(0, elevatorConstants.elevatorPivotTimeout);
    elevatorWinch.configNominalOutputReverse(0, elevatorConstants.elevatorPivotTimeout);
    elevatorWinch.configPeakOutputForward(0.25, elevatorConstants.elevatorPivotTimeout);
    elevatorWinch.configPeakOutputReverse(-0.25, elevatorConstants.elevatorPivotTimeout);
    elevatorWinch.setNeutralMode(NeutralMode.Brake);

    elevatorWinch.configAllowableClosedloopError(elevatorConstants.elevatorSlotIdx, 0, 
    elevatorConstants.elevatorPivotTimeout);
   
   
    leftSolenoid1.set(true);
    leftSolenoid2.set(false);
    rightSolenoid1.set(true);
    rightSolenoid2.set(false);

    setElevatorPID(P, I, D, F);
    P = 0.1;
    I = 0;
    D = 0;
    F = 0;
  }

  public void setElevatorPosition(double desiredPosition) {
    SmartDashboard.putNumber("DesiredPosition", desiredPosition);
    elevatorWinch.set(ControlMode.Position, desiredPosition);
  }
  
  public void deployElevator(){
    // TODO: deploy pneumatic for elevator
     
  }
  

  @Override
  public void periodic() {
    
     SmartDashboard.putNumber("Elevator_1_Pos", elevatorWinch.getSelectedSensorPosition());
  }

  public void setElevatorPID(double P, double I, double D, double F)

  {
    elevatorWinch.config_kP(elevatorConstants.elevatorSlotIdx, P, elevatorConstants.elevatorPivotTimeout);
    elevatorWinch.config_kP(elevatorConstants.elevatorSlotIdx, I, elevatorConstants.elevatorPivotTimeout);
    elevatorWinch.config_kP(elevatorConstants.elevatorSlotIdx, D, elevatorConstants.elevatorPivotTimeout);
    elevatorWinch.config_kP(elevatorConstants.elevatorSlotIdx, F, elevatorConstants.elevatorPivotTimeout);
  }

}
