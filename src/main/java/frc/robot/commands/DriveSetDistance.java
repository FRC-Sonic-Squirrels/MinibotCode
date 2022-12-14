// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;


public class DriveSetDistance extends CommandBase {
  private final DriveSubsystem m_driveSubsystem;
  /** Creates a new DriveSetDistance. */

  public DriveSetDistance(DriveSubsystem driveSubsystem) {
    m_driveSubsystem = driveSubsystem;
    // Use addRequirements() here to declare subsystem dependencies
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      m_driveSubsystem.arcadeDrive(0.5,0.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // We are finished when the distance is greater than our goal.
    return m_driveSubsystem.getDistanceInches() >= 100;
  }
}
