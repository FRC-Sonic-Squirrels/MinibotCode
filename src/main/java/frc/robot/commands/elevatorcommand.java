/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.elevatorSubsystem;

public class elevatorCommand extends CommandBase {
  private static final elevatorSubsystem subsystem = null;
  public elevatorSubsystem m_elevatorSubsystem;
  /**
   * Creates a new elevatorCommand.
   */
  public elevatorCommand() {
    m_elevatorSubsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //RobotContainer.m_elevatorSubsystem.setElevatorPID(0.1, 0, 0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //RobotContainer.m_elevatorSubsystem.setelevatorPosition(0);
    //m_elevatorSubsystem.setElevatorPosition(2048.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
