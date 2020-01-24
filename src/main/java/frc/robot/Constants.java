/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DriveConstants {
        public static final int kLeftNEO = 2;
        public static final int kRightNEO = 1;

        //public static final int kLeftEncoderPort = 3;
        //public static final int kRightEncoderPort = 4;

        public static final boolean kLeftEncoderReversed = false;
        public static final boolean kRightEncoderReversed = false;

        public static final boolean kGyroReversed = true;

        // Actual Minibot width (wheel-to-wheel) 15.625 inches or 0.396875 meters
        public static final double kTrackwidthMeters = 0.396875;
        public static final DifferentialDriveKinematics kDriveKinematics =
                new DifferentialDriveKinematics(kTrackwidthMeters);

        // Determined using frc-characterization tool
        public static final double ksVolts = 0.16;
        public static final double kvVoltSecondsPerMeter = 4.66;
        public static final double kaVoltSecondsSquaredPerMeter = 0.486;

        // Determined using frc-characterization
        public static final double kPDriveVel = .01; // 38.5
        public static final double kDDriveVel = 15.8;

        // SparkMax at 1ms update rate
        // kPDriveVel = 5.65
        // kDDriveVel = 2540.0 

        public static final int kEncoderCPR = 4096;
        public static final double kWheelDiameterMeters = 0.1524;

        // 6 inch (0.1524 meters) traction wheels circumference = 0.1524 * Math.PI;
        public static final double kDistancePerWheelRevolutionMeters =
                kWheelDiameterMeters * Math.PI;

        // gear reduction from NEO to wheels 18:1
        public static final double kGearReduction = 1.0 / 18.0;

        // Assumes the encoders are directly mounted on the motor shafts
        public static final double kEncoderDistancePerPulseMeters =
                (kWheelDiameterMeters * Math.PI * kGearReduction) / (double) kEncoderCPR;

        // https://docs.wpilib.org/en/latest/docs/software/trajectory-end-to-end/entering-constants.html#max-trajectory-velocity-acceleration
        //public static final double kMaxSpeedMetersPerSecond = .1;
        //public static final double kMaxAccelerationMetersPerSecondSquared = .1;

        // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;

    }

    public static final class OIConstants {
        public static final int kDriverController = 0;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = .25;
        public static final double kMaxAccelerationMetersPerSecondSquared = .1;

        // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
    }
}
