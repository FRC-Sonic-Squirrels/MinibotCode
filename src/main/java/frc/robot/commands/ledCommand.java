/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.blinkinSubsystem;

public class ledCommand extends CommandBase {
  /**
   * Creates a new ledCommand.
   */
  blinkinSubsystem m_blinkinSubsystem;

  public ledCommand(blinkinSubsystem subsystem) {
    addRequirements(subsystem);
    m_blinkinSubsystem = subsystem;
}

// Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_blinkinSubsystem.solidOrange();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    /* limelight network table entries used to determine LED colors */
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-one");
    NetworkTableEntry tv = table.getEntry("tv");
    NetworkTableEntry tx = table.getEntry("tx");

    double validTarget = tv.getDouble(0.0);
    double inRange = tx.getDouble(0.0);

    if (validTarget == 1.0) {
      // Values need to be tuned
      if (inRange >= -1 && inRange <= 1.0) {
        m_blinkinSubsystem.solidBlue();
        /* Need to change to dashboard */
        System.out.println("SolidBlue" + inRange);
      }
      else {
        m_blinkinSubsystem.flashingBlue();
         /* Need to change to dashboard */
        System.out.println("flash blue" + inRange);
      }
    }
    else {
      m_blinkinSubsystem.solidOrange();
       /* Need to change to dashboard */
      System.out.println("orange" + validTarget);
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
