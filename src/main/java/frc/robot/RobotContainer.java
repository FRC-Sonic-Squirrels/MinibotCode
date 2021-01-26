/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

/**
 * 
 * Ramsete example code adapted to use NEO motors.
 * 
 * Code adapted from WPILib Example:
 *  https://github.com/wpilibsuite/allwpilib/blob/master/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/ramsetecommand/
 * With some modifications from:
 *  https://github.com/prateekma/2019P2PWorkshop
 * Also see this example:
 *   https://github.com/JimWright4089/FIRSTRobots/blob/master/Code/TWambat/ramsetecommand/
 */

package frc.robot;

import static frc.robot.Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared;
import static frc.robot.Constants.AutoConstants.kMaxSpeedMetersPerSecond;
import static frc.robot.Constants.AutoConstants.kRamseteB;
import static frc.robot.Constants.AutoConstants.kRamseteZeta;
import static frc.robot.Constants.DriveConstants.kDriveKinematics;
import java.util.List;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.driveSubsystem;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */

  // Subsystems
  private final driveSubsystem m_drive = new driveSubsystem();
  
  // Commands

  // Other
   public XboxController m_driverController = new XboxController(OIConstants.kDriverController);

 
  public RobotContainer() {

    // Configure the button bindings
    configureButtonBindings();

    // default command is arcade drive command
    m_drive.setDefaultCommand(
        // A split-stick arcade command, with forward/backward controlled by the left
        // hand, and turning controlled by the right.
        new RunCommand(() -> m_drive.arcadeDrive(0.8 * -m_driverController.getY(GenericHID.Hand.kLeft),
            0.8 * m_driverController.getX(GenericHID.Hand.kRight)), m_drive));
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // Example button
    final JoystickButton ybutton = new JoystickButton(m_driverController, Button.kY.value);
    final JoystickButton startbutton = new JoystickButton(m_driverController, Button.kStart.value);

    // do nothing command
    ybutton.whenPressed(new InstantCommand());
    
    // Start button zeros the gyro heading
    startbutton.whenPressed(() -> m_drive.zeroHeading()); 
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand(String autoName) {

    if (autoName == "donothing") {
      return getNoAutonomousCommand();
    }
    else if (autoName == "figure8") {
      return getAutonomousFigure8Command();
    }
    else if (autoName == "barrel") {
      return getAutonomousBarrelCommand();
    }
    else if (autoName == "slalom") {
      // TODO: write Slalom Auton command and uncomment the next line
      // return getAutonomousSlalomCommand();
    }
    else if (autoName == "bounce") {
      // TODO: write Bounce Auton command and uncomment the next line
      // return getAutonomousBounceCommand();
    }

    // return do nothing if we don't recognize the choice
    System.out.println("Warning: No autonomous command specified.");
    return getNoAutonomousCommand();
  }

  /**
   * getNoAutonomousCommand() - return a trivial command object that does nothing.
   * 
   * This is intended to be a safe placeholder command that does nothing.
   * 
   * @return Autonomous Command object
   */
  public Command getNoAutonomousCommand() {
    // set drive motors power to zero.
    return new RunCommand(() -> m_drive.tankDriveVolts(0, 0));
  }



  /**
   * getAutonomousBarrelCommand - generate Barrel AutoNav Command
   * 
   * @return Command object
   */
  public Command getAutonomousBarrelCommand(){

    Pose2d startPose = new Pose2d(inches2meters(50), inches2meters(90), new Rotation2d(0));
    m_drive.resetOdometry(startPose);

    // TODO: complete Barrel Auton command and uncomment the next line

    // distances are in Meters
    var barrel_path_points = List.of(
      // navigate around first barrel, centered at D5 (150,60)
      new Translation2d( inches2meters(150), inches2meters(90)),
      new Translation2d( inches2meters(180), inches2meters(60)),
      new Translation2d( inches2meters(150), inches2meters(30)),
      new Translation2d( inches2meters(120), inches2meters(60))

      // TODO: navigate around B8  (240, 120)

      // TODO: navigate around D10 (300, 60)

      // TODO: navigate back to start without hitting any markers

      );

    // Start of a Figure 8
    RamseteCommand ramseteCommand = createTrajectoryCommand(
        startPose,
        barrel_path_points,
        new Pose2d(inches2meters(50), inches2meters(90), new Rotation2d(Math.PI)));
    
    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> m_drive.tankDriveVolts(0, 0));
  }

  /**
   * Figure 8 Autonomous Command
   * 
   * @return Command object
   */
  public Command getAutonomousFigure8Command() {
 
    /// distances are in Meters
    var figure_eight = List.of(
      new Translation2d( 0.5, -0.5),
      new Translation2d( 1.0, -1.0),
      new Translation2d( 1.5, -0.5),
      new Translation2d( 1.0,  0.0),
      new Translation2d( 0.5, -0.5),
      new Translation2d( 0.0, -1.0),
      new Translation2d(-0.5, -0.5));

    // Start of a Figure 8
    RamseteCommand ramseteCommand = createTrajectoryCommand(
        new Pose2d(0, 0, new Rotation2d(0)),
        figure_eight,
        new Pose2d(0.0, 0.0, new Rotation2d(0)));
    
    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> m_drive.tankDriveVolts(0, 0));
  }

  /**
   * RamseteCommand - generate a path following autonomous driving command.
   * 
   * @param startPose where the robot starts
   * @param translationList list of intermediate points to pass through
   * @param endPose where the robot stops
   * @return Command object that executes the path
   */
  public RamseteCommand createTrajectoryCommand(Pose2d startPose, List<Translation2d> translationList, Pose2d endPose) {
    DifferentialDriveVoltageConstraint autoVoltageConstraint;
    TrajectoryConfig config;
  
    // Create a voltage constraint to ensure we don't accelerate too fast
    autoVoltageConstraint = new DifferentialDriveVoltageConstraint(m_drive.getFeedforward(), kDriveKinematics, 6);

    // Create config for trajectory
    config = new TrajectoryConfig(kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(kDriveKinematics)
        // Apply the voltage constraint
        .addConstraint(autoVoltageConstraint);

    var initialTime = System.nanoTime();

    // trajectory to follow. All units in meters.
    var trajectory = TrajectoryGenerator.generateTrajectory(
        startPose,
        translationList,
        endPose,
        config);

    RamseteCommand ramseteCommand =
        new RamseteCommand(trajectory, 
            m_drive::getPose,
            new RamseteController(kRamseteB, kRamseteZeta),
            m_drive.getFeedforward(),
            kDriveKinematics,
            m_drive::getWheelSpeeds,
            m_drive.getLeftPidController(),
            m_drive.getRightPidController(),
            m_drive::tankDriveVolts,
            m_drive);

    var dt = (System.nanoTime() - initialTime) / 1E6;
    System.out.println("RamseteCommand generation time: " + dt + "ms");

    // Run path following command, then stop at the end.
    return ramseteCommand;
  }

  /**
   * Convert units form inches to meters
   * 
   * @param distInches
   * @return distance in meters
   */
  public static double inches2meters(double distInches) {
    return distInches * 0.0254;
  }
}
