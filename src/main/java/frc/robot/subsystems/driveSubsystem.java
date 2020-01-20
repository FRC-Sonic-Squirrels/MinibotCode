/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.EncoderType;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.Constants.DriveConstants;


public class driveSubsystem extends SubsystemBase {
  public static final CANSparkMax m_leftNEO =
      new CANSparkMax(DriveConstants.kLeftNEO, MotorType.kBrushless);
  public static final CANSparkMax m_rightNEO =
      new CANSparkMax(DriveConstants.kRightNEO, MotorType.kBrushless);
  public static SpeedController m_leftMotors = new SpeedControllerGroup(m_leftNEO);;
  public static SpeedController m_rightMotors = new SpeedControllerGroup(m_leftNEO);
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);
  private final CANEncoder m_leftEncoder = m_leftNEO.getEncoder(EncoderType.kQuadrature, 4096);
  private final CANEncoder m_rightEncoder = m_rightNEO.getEncoder(EncoderType.kQuadrature, 4096);

  // The gyro sensor
  //private final Gyro m_gyro = new ADXRS450_Gyro();
  private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;

  /**
   * driveSubsystem constructor
   * 
   * uses NEOs and implements odometry
   */
  public driveSubsystem() {
    // set all NEOs to factory defaults
    m_leftNEO.restoreFactoryDefaults();
    m_rightNEO.restoreFactoryDefaults();

    // set scaling factor for CANEncoder.getPossition() so that it matches the output of
    // Encoder.getDistance() method.
    m_leftEncoder.setPositionConversionFactor(
        DriveConstants.kDistancePerWheelRevolutionMeters * DriveConstants.kGearReduction);
    m_rightEncoder.setPositionConversionFactor(
        DriveConstants.kDistancePerWheelRevolutionMeters * DriveConstants.kGearReduction);

    // Native scale is RPM. Scale velocity so that it is in meters/sec
    m_leftEncoder.setVelocityConversionFactor(
        DriveConstants.kDistancePerWheelRevolutionMeters * DriveConstants.kGearReduction / 60.0);
    m_rightEncoder.setVelocityConversionFactor(
        DriveConstants.kDistancePerWheelRevolutionMeters * DriveConstants.kGearReduction / 60.0);

    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));

    //SmartDashboard.putString("FirmwareVersion", m_gyro.getFirmwareVersion());
  }

  @Override
  public void periodic() {
    // Note: periodic() is run by the scheduler, always. No matter what.
    // Update the odometry in the periodic block
    m_odometry.update(Rotation2d.fromDegrees(getHeading()), m_leftEncoder.getPosition(),
        m_rightEncoder.getPosition());

    //SmartDashboard.putBoolean("IMU_Connected", m_gyro.isConnected());
    //SmartDashboard.putBoolean("IMU_IsCalibrating", m_gyro.isCalibrating());
    SmartDashboard.putNumber("IMU_Yaw", m_gyro.getYaw());
    //SmartDashboard.putNumber("IMU_Pitch", m_gyro.getPitch());
    //SmartDashboard.putNumber("IMU_Roll", m_gyro.getRoll());

    /* Display 9-axis Heading (requires magnetometer calibration to be useful) */
    //SmartDashboard.putNumber("IMU_CompassHeading", m_gyro.getCompassHeading());
    SmartDashboard.putNumber("IMU_FusedHeading", m_gyro.getFusedHeading());

    /* These functions are compatible w/the WPI Gyro Class, providing a simple */
    /* path for upgrading from the Kit-of-Parts gyro to the navx-MXP */

    //SmartDashboard.putNumber("IMU_TotalYaw", m_gyro.getAngle());
    //SmartDashboard.putNumber("IMU_YawRateDPS", m_gyro.getRate());

  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getVelocity(),
        m_rightEncoder.getVelocity());
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotors.setVoltage(leftVolts);
    m_rightMotors.setVoltage(-rightVolts);
  }

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  public void resetEncoders() {
    m_leftEncoder.setPosition(0.0); // was Encoder.reset();
    m_rightEncoder.setPosition(0.0); // was Encoder.reset();
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    // Was Encoder.getDistance()
    return (m_leftEncoder.getPosition() + m_rightEncoder.getPosition()) / 2.0;
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public CANEncoder getLeftEncoder() {
    return m_leftEncoder;
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public CANEncoder getRightEncoder() {
    return m_rightEncoder;
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /**
   * Zeroes the heading of the robot.
   */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from 180 to 180
   */
  public double getHeading() {
    // TODO: use get fused heading 
    return Math.IEEEremainder(m_gyro.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }
}
