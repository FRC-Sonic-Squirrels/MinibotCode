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
    
    /* Use limelight network table entries for LED on/near target values */
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-one");
    NetworkTableEntry tv = table.getEntry("tv");
    NetworkTableEntry tx = table.getEntry("tx");

    double validTarget = tv.getDouble(0.0);
    double inShootingRange = tx.getDouble(0.0);

    if (validTarget == 1.0) {
      if (inShootingRange >= -1 && inShootingRange <= 1.0) {
        m_blinkin.solidBlue();
        /* Need to change to dashboard */
        System.out.println("SolidBlue" + inShootingRange);
      }
      else {
        m_blinkin.flashingBlue();
         /* Need to change to dashboard */
        System.out.println("flash blue" + inShootingRange);
      }
    }
    else {
      m_blinkin.solidOrange();
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
