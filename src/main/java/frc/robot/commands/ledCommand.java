/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.blinkin;

public class ledCommand extends CommandBase {
  /**
   * Creates a new ledCommand.
   */
  blinkin m_blinkin;
  
  public ledCommand(blinkin subsystem) {
    addRequirements(subsystem);
    m_blinkin = subsystem;
}

// Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_blinkin.solidOrange();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println( "ledCommand is being called");
    if (m_blinkin.limeLightOnValidTarget()) {
      if (m_blinkin.limeLightInShootingRange()) {
        m_blinkin.solidBlue();
      } 
      else {
        m_blinkin.flashingBlue();
      }
    }
    else {
      m_blinkin.solidOrange();
    }

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
