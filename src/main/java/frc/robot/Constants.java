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
    public static final double driverSpeedSensitivityDefault = 1;
    public static final double driverTurnSensitivity = 1;

    // public enum ModeCount {
    //   RunningToTake(0),
    //   Catching(1),
    //   LiftingExtending(2),
    //   Putting(3);
    
    //   private final int code;
    //   private ModeCount(int code) {
    //     this.code = code;
    //   }
    
    //   public int getCode() {
    //     return code;
    //   }

    //   public static ModeCount getType(final int id) {
    //     ModeCount[] types = ModeCount.values();
    //     for (ModeCount type : types) {
    //         if (type.getCode() == id) {
    //             return type;
    //         }
    //     }
    //     return null;
    //   }
    // }
  }

  public static final class DriveConstants {
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

    public static final double DRIVE_VELOCITY_KP = 1.7936;

    public static final double TRACK_WIDTH = 0.555;
    public static final DifferentialDriveKinematics KINEMATICS =
      new DifferentialDriveKinematics(TRACK_WIDTH);

    public static final double FEED_FORWARD_KS = 0.15053;
    public static final double FEED_FORWARD_KV = 2.7446;
    public static final double FEED_FORWARD_KA = 0.21353;

    public static final double kOffBalanceAngleThresholdDegrees = 10.0;
    public static final double kOonBalanceAngleThresholdDegrees = 5.0;
  }

  public static final class LiftConstants {
    public static final int RightMotorID = 5;
    public static final int LeftMotorID = 6;

    //TODO: tune the feedforward gain(measuring on the extended position)
    public static final double kP = 1;    //Propotion value for feedforward

    public static final int EncoderPPR = 1;
    //TODO: change the gear ratio
    public static final int LiftGearRatio = 16;

    // measured from the horizontal
    //TODO: measure the position with a little tolerance & changing the tolerance
    public static final double LiftExtendedPos = 1.09; //[rad]
    public static final double LiftHorizontalPos = 0.0; //[rad]
    public static final double Tolerance = 0.04;       //[rad]

    public enum LiftOperateMode {
      Up,
      Down,
      Stay
    }
  }

  public static final class ArmRotationConstants {
    // the conversion will be dealt within the encoder class / no need to create new conversion factor
    public static final int ID = 7;
    
    public static final int kEncoderPPR = 1;
    public static final int ArmGearRatio = 100;

    public static final double ArmButtomRads = 0.0;         //[rad]
    public static final double ArmOutsideMaxRads = 2.023;
    public static final double ArmInsideMaxRads = -2.348;
    public static final double ArmForwardVertRads = 1.548;   //[rad]
    public static final double ArmBackwardVertRads = -2.005;   //[rad]
    public static final double TestRads = 0.5;
    public static final double Tolerance = 0.1;             //[rad]
  }

  public static final class ArmCatchConstants {
    // the conversion will be dealt within the encoder class / no need to create new conversion factor
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

    public static final double kMaxVelocityMeterPerSecond = 3;          //[m/s]
    public static final double kMaxAccelerationMeterPerSecSquared = 10; //[m/s*2]

    public static final int kEncoderPPR = 1;
    public static final int ArmCatchGearRatio = 1;
    public static final double wheelDiameter = 0.1; //[m]
    public static final double kEncoderDistancePerPulse = wheelDiameter * Math.PI / (double)(kEncoderPPR*ArmCatchGearRatio); //[m]

    // 左右のモーターともに、近づくとPoseが増加、遠ざかるとPoseが減少、するようにする
    public static final double ArmFarPose = 0.0;      //[m] 最大の位置
    public static final double ArmNearPose = 3.0;     //[m]  最小の位置
    public static final double ArmLeftMediumPose = 1.0;
    public static final double Tolerance = 0.02;      //[m]
    public static final double AccelerationThreshold = -0.1;  // アームの物体衝突を検知する閾値

    public static final double mediumSetPoint = 2.0;  //[m]
    public static final double defaultSetPoint = 0.4; //[m]
    
    public enum State {
      Disabled(0),
      Reached(1),
      UnReached(2);
    
      private final int code;
      private State(int code) {
        this.code = code;
      }
    
      public int getCode() {
        return code;
      }
    }

    public enum ArmOperatorMode {
      ReturnToMedium,
      ReturnToDefault
    }
  }

  public static final class SliderConstants {
    public static final int RightMotorID = 10;
    public static final int LeftMotorID = 11;

    public static final double kP = 1;
    public static final double kI = 0;
    public static final double kD = 0;

    // These are fake gains; in actuality these must be determined individually for each robot
    public static final double kSVolts = 1;
    public static final double kGVolts = 1;
    public static final double kVVoltSecondPerRad = 0.5;
    public static final double kAVoltSecondSquaredPerRad = 0.1;

    public static final double kMaxVelocityMeterPerSecond = 0.05;
    public static final double kMaxAccelerationMeterPerSecSquared = 0.01;

    public static final int kEncoderPPR = 1;
    public static final int SliderGearRatio = 16;
    //TODO: measure the diameter
    public static final double wheelDiameter = 0.0130; //[m]
    public static final double kEncoderDistancePerPulse = wheelDiameter * Math.PI / (double)(kEncoderPPR*SliderGearRatio); //[m]

    public static final double SliderShortestInMeters = 0.0;  //[m] set the shortest as 0
    public static final double SliderLongestInMeters = 1.0;   //[m] maximum distance
    public static final double Tolerance = 0.02;               //[m]

    public enum SliderOperateMode {
      ExtendAll,
      ShrinkAll,
      CustomSetPoint,
      Stay
    }
  }
  public static final class AutoConstants {
    public static final double MaxSpeedMetersPerSecond = 0.5;
    public static final double MaxAccelerationMetersPerSecondSquared = 0.5;

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double RAMSETE_B = 2;
    public static final double RAMSETE_ZETA = 0.7;
  }
  
  public static final class TestConstants {
    public static final int ArmCatchRightID = 5;
    public static final int ArmCatchLeftID = 6;
    public static final int ArmRotationID = 7;
  }

  public enum Direction {
    Left(0),
    Right(1);

    private final int code;
    private Direction(int code){
      this.code = code;
    }
    public int getCode() {
      return code;
    }
  }
}
