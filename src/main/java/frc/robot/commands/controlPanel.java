/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.colorSensor;
import frc.robot.subsystems.controlPanelMotors;

public class controlPanel extends CommandBase {
  private colorSensor m_colorSensor;
  private controlPanelMotors m_controlPanelMotor;
  /**
   * Creates a new controlPanel.
   */
  public controlPanel(colorSensor cs, controlPanelMotors cp) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_controlPanelMotor = cp;
    m_colorSensor = cs;
    addRequirements(m_controlPanelMotor, m_colorSensor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // TODO: motor needs to contact control panel first
    m_controlPanelMotor.setSpeed(0.2);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_controlPanelMotor.setSpeed(0);
    // TODO: retract motor
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_colorSensor.getRotationCount() >= 3) {
      return true;
    }
    return false;
  }
}
