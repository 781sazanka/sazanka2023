// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int DriverControllerPort = 0;
    public static final int MechanicsControllerPort = 1;
  }

  public static final class Drivetrain {
    public static final int MOTOR_RIGHT_1 = 1;
    public static final int MOTOR_RIGHT_2 = 2;
    public static final int MOTOR_LEFT_1 = 3;
    public static final int MOTOR_LEFT_2 = 4;

    public static final double SENSOR_UNITS_PER_REV = 2048;
    public static final double WHEEL_DIAMETER = Units.inchesToMeters(6);
    public static final double GEAR_RATIO = 12.75;
    public static final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
    public static final double SENSOR_UNITS_PER_METER =
            (SENSOR_UNITS_PER_REV * GEAR_RATIO) / WHEEL_CIRCUMFERENCE;

    public static final double DRIVE_VELOCITY_KP = 5.0;

    public static final double TRACK_WIDTH = 555e-4;
    public static final DifferentialDriveKinematics KINEMATICS =
            new DifferentialDriveKinematics(TRACK_WIDTH);

    public static final double FEED_FORWARD_KS = 0.14537;
    public static final double FEED_FORWARD_KV = 2.2311;
    public static final double FEED_FORWARD_KA = 0.52691;
  }

  public static final class LiftConstants {
    public static final int RightMotorID = 5;
    public static final int LeftMotorID = 6;

    public static final double kP = 1;
    public static final double kI = 0;
    public static final double kD = 0;

    // These are fake gains; in actuality these must be determined individually for each robot
    public static final double kSVolts = 1;
    public static final double kGVolts = 1;
    public static final double kVVoltSecondPerRad = 0.5;
    public static final double kAVoltSecondSquaredPerRad = 0.1;

    public static final double kMaxVelocityRadPerSecond = 3;
    public static final double kMaxAccelerationRadPerSecSquared = 10;

    public static final int EncoderPPR = 1;
    public static final int LiftGearRatio = 16;
    public static final double kEncoderDistancePerPulse = 2.0 * Math.PI / (double)(EncoderPPR*LiftGearRatio);

    // The offset of the arm from the horizontal in its neutral position,
    // measured from the horizontal
    public static final double LiftOffsetRads = 0.0; //in case / this isn't in use
    public static final double LiftGoalPositionRad = 2.0; //radian
    public static final double Tolerance = 0.2; // in radians
  }

  public static final class ArmRotationConstants {
    public static final int ID = 7;

    public static final double kP = 1;
    public static final double kI = 0;
    public static final double kD = 0;

    // These are fake gains; in actuality these must be determined individually for each robot
    public static final double kSVolts = 1;
    public static final double kGVolts = 1;
    public static final double kVVoltSecondPerRad = 0.5;
    public static final double kAVoltSecondSquaredPerRad = 0.1;
    
    // needs to be going through a conversion factor
    public static final double kMaxVelocityRadPerSecond = 3;
    public static final double kMaxAccelerationRadPerSecSquared = 10;

    public static final boolean kEncoderReversed = false;
    public static final int kEncoderPPR = 1;
    public static final int ArmGearRatio = 16;
    //unused / calculating the liner distance from radians
    public static final double kEncoderDistancePerPulse = 2.0 * Math.PI / (double)(kEncoderPPR*ArmGearRatio);

    // The offset of the arm from the horizontal in its neutral position,
    // measured from the horizontal in [radian]
    // needs to be going through a conversion factor
    public static final double ArmOffsetRads = 0.0;
    public static final double ArmVertPosRads = 2.0;
    public static final double ArmTargetRads = 3.0;
    public static final double Tolerance = 0.2;
    
    //TODO: modifying the conversion factor 
    // converting actual arm rotation rads to motor rotation rads
    public static <T extends Number> T ArmConversionFactor(T value) {
      double converted = 2.0 * value.doubleValue();
      // You can also use value.getClass() to get the class of the input value
      // and use that to create a new instance of the same class for the return value
      // For example:
      // return (T) value.getClass().getConstructor(double.class).newInstance(converted);
      return (T) Double.valueOf(converted); // You can also return the value as a Double object
    }

  }

  public static final class SliderConstants {
    public static final int RightID = 8;
    public static final int LeftID = 9;

    public static final double kP = 1;
    public static final double kI = 0;
    public static final double kD = 0;

    // These are fake gains; in actuality these must be determined individually for each robot
    public static final double kSVolts = 1;
    public static final double kGVolts = 1;
    public static final double kVVoltSecondPerRad = 0.5;
    public static final double kAVoltSecondSquaredPerRad = 0.1;

    public static final double kMaxVelocityRadPerSecond = 3;
    public static final double kMaxAccelerationRadPerSecSquared = 10;

    public static final boolean kEncoderReversed = false;
    public static final int kEncoderPPR = 1;
    public static final int SliderGearRatio = 16;
    //unused / calculating the liner distance from radians
    public static final double kEncoderDistancePerPulse = 2.0 * Math.PI / (double)(kEncoderPPR*SliderGearRatio);

    // The offset of the arm from the horizontal in its neutral position,
    // measured from the horizontal in radian
    public static final double SliderOffsetRads = 0.0;
    public static final double SliderVertPosRads = 2.0;
    public static final double SliderTargetRads = 3.0;
  }
  public static final class AutoConstants {
    public static final double MaxSpeedMetersPerSecond = 0.8;
    public static final double MaxAccelerationMetersPerSecondSquared = 0.2;

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double RAMSETE_B = 2;
    public static final double RAMSETE_ZETA = 0.7;
  }
  
}
